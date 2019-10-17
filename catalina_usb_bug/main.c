//
//  main.c
//  catalina_usb_bug
//
//  Created by Yoctopuce on 17.10.19.
//  Copyright © 2019 Yoctopuce. All rights reserved.
//
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <fcntl.h>
#include <unistd.h>

static USB_THREAD_STATE get_usb_thread_state(yContextSt  *ctx)
{
    USB_THREAD_STATE res;
    yEnterCriticalSection(&ctx->parano_cs);
    res =ctx->usb_thread_state;
    yLeaveCriticalSection(&ctx->parano_cs);
    return res;
}


static void set_usb_thread_state(yContextSt  *ctx, USB_THREAD_STATE st)
{
    yEnterCriticalSection(&ctx->parano_cs);
    ctx->usb_thread_state = st;
    yLeaveCriticalSection(&ctx->parano_cs);
}

static void *event_thread(void *param)
{
    yContextSt  *ctx=param;
    
    ctx->usb_run_loop     = CFRunLoopGetCurrent();
    set_usb_thread_state(ctx, USB_THREAD_RUNNING);
    /* Non-blocking. See if the OS has any reports to give. */
    HALLOG("Start event_thread run loop\n");
    while (get_usb_thread_state(ctx) != USB_THREAD_MUST_STOP) {
        CFRunLoopRunInMode( kCFRunLoopDefaultMode, 10, FALSE);
    }
    
    HALLOG("event_thread run loop stopped\n");
    
    set_usb_thread_state(ctx, USB_THREAD_STOPED);
    return NULL;
}


static int setupHIDManager(yContextSt *ctx, OSX_HID_REF *hid, char *errmsg)
{
    int             c_vendorid = YOCTO_VENDORID;
    CFMutableDictionaryRef dictionary;
    CFNumberRef     Vendorid;
    IOReturn        tIOReturn;
    
    yInitializeCriticalSection(&hid->hidMCS);
    // Initialize HID Manager
    hid->manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    // create dictionary to match Yocto devices
    dictionary = CFDictionaryCreateMutable(kCFAllocatorDefault,1,&kCFTypeDictionaryKeyCallBacks,&kCFTypeDictionaryValueCallBacks);
    Vendorid = CFNumberCreate( kCFAllocatorDefault, kCFNumberIntType, &c_vendorid );
    CFDictionarySetValue( dictionary, CFSTR( kIOHIDVendorIDKey ), Vendorid );
    CFRelease(Vendorid);
    // register the dictionary
    IOHIDManagerSetDeviceMatching(hid->manager, dictionary );
    // now we can release the dictionary
    CFRelease(dictionary);
    // schedule the HID Manager with our global run loop
    IOHIDManagerScheduleWithRunLoop(hid->manager, ctx->usb_run_loop, kCFRunLoopDefaultMode);
    
    // Now open the IO HID Manager reference
    tIOReturn = IOHIDManagerOpen(hid->manager, kIOHIDOptionsTypeNone );
    if(kIOReturnSuccess != tIOReturn ||CFGetTypeID(hid->manager) != IOHIDManagerGetTypeID()){
        HALLOG("Unable to Open HID Manager");
        return YERRMSG(YAPI_NOT_SUPPORTED,"Unable to Open HID Manager");
    }
    return YAPI_SUCCESS;
    
}



static void stopHIDManager(OSX_HID_REF *hid)
{
    if (hid->manager) {
        IOHIDManagerClose(hid->manager, kIOHIDOptionsTypeNone );
        CFRelease( hid->manager);
        hid->manager=NULL;
        yDeleteCriticalSection(&hid->hidMCS);
    }
}


int yyyUSB_init(yContextSt *ctx,char *errmsg)
{
   
}


int yyyUSB_stop(yContextSt *ctx,char *errmsg)
{
    stopHIDManager(&ctx->hid);
    
    if(get_usb_thread_state(ctx) == USB_THREAD_RUNNING){
        set_usb_thread_state(ctx, USB_THREAD_MUST_STOP);
        CFRunLoopStop(ctx->usb_run_loop);
    }
    pthread_join(ctx->usb_thread,NULL);
    YASSERT(get_usb_thread_state(ctx) == USB_THREAD_STOPED);
    
    yReleaseGlobalAccess(ctx);
    yDeleteCriticalSection(&ctx->parano_cs);
    return 0;
}




static u32 get_int_property(IOHIDDeviceRef device, CFStringRef key)
{
    CFTypeRef ref;
    u32 value;
    
    ref = IOHIDDeviceGetProperty(device, key);
    if (ref) {
        if (CFGetTypeID(ref) == CFNumberGetTypeID() && CFNumberGetValue((CFNumberRef) ref, kCFNumberSInt32Type, &value)) {
            return value;
        }
    }
    return 0;
}


static void get_txt_property(IOHIDDeviceRef device,char *buffer,u32 maxlen, CFStringRef key)
{
    CFTypeRef ref;
    size_t  len;
    
    ref = IOHIDDeviceGetProperty(device, key);
    if (ref) {
        if (CFGetTypeID(ref) == CFStringGetTypeID()) {
            const char *str;
            CFStringEncoding encodingMethod;
            encodingMethod = CFStringGetSystemEncoding();
            // 1st try for English system
            str = CFStringGetCStringPtr(ref, encodingMethod);
            //str = NULL;
            if ( str == NULL ) {
                // 2nd try
                encodingMethod = kCFStringEncodingUTF8;
                str = CFStringGetCStringPtr(ref, encodingMethod);
            }
            if( str == NULL ) {
                //3rd try
                CFIndex cflength = CFStringGetLength(ref)*2+2;
                char *tmp_str = yMalloc( (u32)cflength);
                if (!CFStringGetCString(ref, tmp_str, cflength, kCFStringEncodingUTF8 )) {
                    yFree( tmp_str );
                    *buffer=0;
                    return;
                }
                if(cflength>maxlen-1){
                    cflength=maxlen-1;
                }
                memcpy(buffer,tmp_str,cflength);
                buffer[cflength]=0;
                yFree( tmp_str );
                return;
            }
            len=strlen(str);
            if(len>maxlen-1){
                len=maxlen-1;
            }
            memcpy(buffer,str,len);
            buffer[len]=0;
            return;
        }
    }
    *buffer=0;
}





static IOHIDDeviceRef* getDevRef(OSX_HID_REF *hid, CFIndex *deviceCount)
{
    
    CFSetRef        deviceCFSetRef;
    IOHIDDeviceRef  *dev_refs=NULL;
    
    *deviceCount = 0;
    
    yEnterCriticalSection(&hid->hidMCS);
    deviceCFSetRef = IOHIDManagerCopyDevices(hid->manager);
    yLeaveCriticalSection(&hid->hidMCS);
    if (deviceCFSetRef!= NULL) {
        // how many devices in the set?
        *deviceCount = CFSetGetCount( deviceCFSetRef );
        dev_refs = yMalloc( sizeof(IOHIDDeviceRef) * (u32)*deviceCount );
        // now extract the device ref's from the set
        CFSetGetValues( deviceCFSetRef, (const void **) dev_refs );
    }
    return dev_refs;
}


int yyyUSBGetInterfaces(yInterfaceSt **ifaces,int *nbifaceDetect,char *errmsg)
{
    int             nbifaceAlloc;
    int             deviceIndex;
    CFIndex         deviceCount;
    IOHIDDeviceRef  *dev_refs;
    
    // get all device detected by the OSX
    dev_refs = getDevRef(&yContext->hid, &deviceCount);
    if(dev_refs == NULL) {
        return 0;
    }
    
    // allocate buffer for detected interfaces
    *nbifaceDetect = 0;
    nbifaceAlloc  = 8;
    *ifaces =yMalloc(nbifaceAlloc * sizeof(yInterfaceSt));
    memset(*ifaces, 0 ,nbifaceAlloc * sizeof(yInterfaceSt));
    for(deviceIndex=0 ; deviceIndex < deviceCount ;deviceIndex++){
        u16 vendorid;
        u16 deviceid;
        IOHIDDeviceRef dev = dev_refs[deviceIndex];
        yInterfaceSt    *iface;
        vendorid = get_int_property(dev,CFSTR(kIOHIDVendorIDKey));
        deviceid = get_int_property(dev,CFSTR(kIOHIDProductIDKey));
        //ensure the buffer of detected interface is big enought
        if(*nbifaceDetect == nbifaceAlloc){
            yInterfaceSt    *tmp;
            tmp = (yInterfaceSt*) yMalloc(nbifaceAlloc*2 * sizeof(yInterfaceSt));
            memset(tmp,0,nbifaceAlloc*2 * sizeof(yInterfaceSt));
            yMemcpy(tmp,*ifaces, nbifaceAlloc * sizeof(yInterfaceSt) );
            yFree(*ifaces);
            *ifaces = tmp;
            nbifaceAlloc    *=2;
        }
        iface = *ifaces + *nbifaceDetect;
        //iface->devref   = dev;
        iface->vendorid = vendorid;
        iface->deviceid = deviceid;
        get_txt_property(dev,iface->serial,YOCTO_SERIAL_LEN*2, CFSTR(kIOHIDSerialNumberKey));
        HALENUMLOG("work on interface %d (%x:%x:%s)\n",deviceIndex,vendorid,deviceid,iface->serial);
        (*nbifaceDetect)++;
    }
    yFree(dev_refs);
    return YAPI_SUCCESS;
}


/*****************************************************************
 * OSX implementation of yyypacket functions
 *****************************************************************/




// return 1 if OS hdl are identical
//        0 if any of the interface has changed
int yyyOShdlCompare( yPrivDeviceSt *dev, yInterfaceSt *newdev)
{
    if(dev->infos.nbinbterfaces != 1){
        HALLOG("bad number of inteface for %s (%d)\n",dev->infos.serial,dev->infos.nbinbterfaces);
        return 0;
    }
    return 1;
}




static void Handle_IOHIDDeviceIOHIDReportCallback(
                                                  void *          inContext,          // context from IOHIDDeviceRegisterInputReportCallback
                                                  IOReturn        inResult,           // completion result for the input report operation
                                                  void *          inSender,           // IOHIDDeviceRef of the device this report is from
                                                  IOHIDReportType inType,             // the report type
                                                  uint32_t        inReportID,         // the report ID
                                                  uint8_t *       inReport,           // pointer to the report data
                                                  CFIndex         InReportLength)     // the actual size of the input report
{
    yInterfaceSt *iface= (yInterfaceSt*) inContext;
    yPktQueuePushD2H(iface,&iface->tmprxpkt,NULL);
    memset(&iface->tmprxpkt,0xff,sizeof(USB_Packet));
}


int yyySetup(yInterfaceSt *iface,char *errmsg)
{
    char str[32];
    int i;
    CFIndex deviceCount;
    IOHIDDeviceRef *dev_refs;
    
    
    if (yContext->osx_flags & YCTX_OSX_MULTIPLES_HID) {
        if (YISERR(setupHIDManager(yContext, &iface->hid,errmsg))) {
            return YAPI_IO_ERROR;
        }
        // get all device detected by the OSX
        dev_refs = getDevRef(&iface->hid, &deviceCount);
    } else {
        dev_refs = getDevRef(&yContext->hid, &deviceCount);
    }
    if(dev_refs == NULL) {
        return YERRMSG(YAPI_IO_ERROR,"Device disappear before yyySetup");
    }
    
    
    for(i=0 ; i < deviceCount ;i++){
        u16 vendorid;
        u16 deviceid;
        IOHIDDeviceRef dev = dev_refs[i];
        vendorid = get_int_property(dev,CFSTR(kIOHIDVendorIDKey));
        deviceid = get_int_property(dev,CFSTR(kIOHIDProductIDKey));
        if (iface->vendorid == vendorid && iface->deviceid == deviceid){
            char serial[YOCTO_SERIAL_LEN * 2];
            memset(serial, 0, YOCTO_SERIAL_LEN * 2);
            get_txt_property(dev,serial,YOCTO_SERIAL_LEN * 2, CFSTR(kIOHIDSerialNumberKey));
            if (YSTRCMP(serial, iface->serial) == 0){
                HALLOG("right Interface detected (%x:%x:%s)\n",vendorid,deviceid,iface->serial);
                iface->devref = dev;
                break;
            }
        }
    }
    yFree(dev_refs);
    if (i == deviceCount) {
        return YERRMSG(YAPI_IO_ERROR,"Unable to match device detected");
    }
    
    IOReturn ret = IOHIDDeviceOpen(iface->devref, kIOHIDOptionsTypeNone);
    if (ret != kIOReturnSuccess) {
        YSPRINTF(str,32,"Unable to open device (0x%x)",ret);
        return YERRMSG(YAPI_IO_ERROR,str);
    }
    
    yPktQueueInit(&iface->rxQueue);
    yPktQueueInit(&iface->txQueue);
    
    
    /* Create the Run Loop Mode for this device. printing the reference seems to work. */
    sprintf(str, "yocto_%p", iface->devref);
    iface->run_loop_mode = CFStringCreateWithCString(NULL, str, kCFStringEncodingASCII);
    /* Attach the device to a Run Loop */
    IOHIDDeviceScheduleWithRunLoop(iface->devref, yContext->usb_run_loop, iface->run_loop_mode);
    IOHIDDeviceRegisterInputReportCallback( iface->devref,              // IOHIDDeviceRef for the HID device
                                           (u8*) &iface->tmprxpkt,     // pointer to the report data
                                           USB_PKT_SIZE,               // number of bytes in the report (CFIndex)
                                           &Handle_IOHIDDeviceIOHIDReportCallback,   // the callback routine
                                           iface);                     // context passed to callback
    
    // save setup ed iface pointer in context in order
    // to retrieve it during unplugcallback
    for (i=0; i< SETUPED_IFACE_CACHE_SIZE ; i++) {
        if(yContext->setupedIfaceCache[i]==NULL){
            yContext->setupedIfaceCache[i] = iface;
            break;
        }
    }
    if (i==SETUPED_IFACE_CACHE_SIZE) {
        return YERRMSG(YAPI_IO_ERROR,"Too many setup USB interfaces");
    }
    iface->flags.yyySetupDone = 1;
    return 0;
}



int yyySignalOutPkt(yInterfaceSt *iface, char *errmsg)
{
    int res =YAPI_SUCCESS;
    pktItem *pktitem;
    
    yPktQueuePopH2D(iface, &pktitem);
    while (pktitem!=NULL){
        if(iface->devref==NULL){
            yFree(pktitem);
            return YERR(YAPI_IO_ERROR);
        }
        res = IOHIDDeviceSetReport(iface->devref,
                                   kIOHIDReportTypeOutput,
                                   0, /* Report ID*/
                                   (u8*)&pktitem->pkt, sizeof(USB_Packet));
        yFree(pktitem);
        if (res != kIOReturnSuccess) {
            dbglog("IOHIDDeviceSetReport failed with 0x%x\n", res);
            return YERRMSG(YAPI_IO_ERROR,"IOHIDDeviceSetReport failed");;
        }
        yPktQueuePopH2D(iface, &pktitem);
    }
    return YAPI_SUCCESS;
}



void yyyPacketShutdown(yInterfaceSt  *iface)
{
    int i;
    
    // remove iface from setup ifaces
    for (i=0; i< SETUPED_IFACE_CACHE_SIZE ; i++) {
        if(yContext->setupedIfaceCache[i]==iface){
            yContext->setupedIfaceCache[i] = NULL;
            break;
        }
    }
    YASSERT(i<SETUPED_IFACE_CACHE_SIZE);
    if(iface->devref!=NULL){
        IOHIDDeviceRegisterInputReportCallback(iface->devref,              // IOHIDDeviceRef for the HID device
                                               (u8*) &iface->tmprxpkt,   // pointer to the report data (uint8_t's)
                                               USB_PKT_SIZE,              // number of bytes in the report (CFIndex)
                                               NULL,   // the callback routine
                                               iface);                      // context passed to callback
        IOHIDDeviceClose(iface->devref, kIOHIDOptionsTypeNone);
        iface->devref=NULL;
    }
    yPktQueueFree(&iface->rxQueue);
    yPktQueueFree(&iface->txQueue);
    iface->flags.yyySetupDone = 0;
    CFRelease(iface->run_loop_mode);
    if (yContext->osx_flags & YCTX_OSX_MULTIPLES_HID) {
        stopHIDManager(&iface->hid);
    }
}

#endif

#ifdef IOS_API
#include "yproto.h"

int yyyUSB_init(yContextSt *ctx,char *errmsg)
{
    return YAPI_SUCCESS;
}

int yyyUSB_stop(yContextSt *ctx,char *errmsg)
{
    return 0;
}

int yyyUSBGetInterfaces(yInterfaceSt **ifaces,int *nbifaceDetect,char *errmsg)
{
    *nbifaceDetect = 0;
    return 0;
}

int yyyOShdlCompare( yPrivDeviceSt *dev, yInterfaceSt *newdev)
{
    return 1;
}

int yyySetup(yInterfaceSt *iface,char *errmsg)
{
    return YERR(YAPI_NOT_SUPPORTED);
}

int yyySignalOutPkt(yInterfaceSt *iface, char *errmsg)
{
    return -1;
}

void yyyPacketShutdown(yInterfaceSt  *iface)
{}

#endif


#include <stdio.h>

int main(int argc, const char * argv[]) {
   
    
    char str[256];
    size_t size = sizeof(str);
    
    if (sysctlbyname("kern.osrelease", str, &size, NULL, 0) ==0){
        int numver;
        //15.x.x  OS X 10.11.x El Capitan
        //14.x.x  OS X 10.10.x Yosemite
        //13.x.x  OS X 10.9.x Mavericks
        //12.x.x  OS X 10.8.x Mountain Lion
        //11.x.x  OS X 10.7.x Lion
        //10.x.x  OS X 10.6.x Snow Leopard
        str[2]=0;
        numver = atoi(str);
        if (numver >= 13 && numver < 15){
            ctx->osx_flags |= YCTX_OSX_MULTIPLES_HID;
        }
    }
    
    yInitializeCriticalSection(&ctx->parano_cs);
    set_usb_thread_state(ctx, USB_THREAD_NOT_STARTED);
    pthread_create(&ctx->usb_thread, NULL, event_thread, ctx);
    while(get_usb_thread_state(ctx) != USB_THREAD_RUNNING){
        usleep(50000);
    }
    
    if (YISERR(setupHIDManager(ctx, &ctx->hid,errmsg))) {
        return YAPI_IO_ERROR;
    }
    
    
    
    int res;
    int nbifaceDetect, i;
    libusb_context      *libusb;
    const struct libusb_version *libusb_v;
    libusb_device   **list;
    ssize_t         nbdev;
    int             returnval = 0;
    unsigned             alloc_size;
    yInterfaceSt    *iface, *ifaces;
    
    printf("this is a simple program that exhibit a bug in the USB stack\n");
    printf("of the Raspberry PI Zero and any Yoctopuce device\n");
    
    if (argc > 1 && argv[1]){
        verbose = 1;
    }
    
    // init libUSB
    res = libusb_init(&libusb);
    if(res != 0) {
        return ySetErr("Unable to start lib USB", res);
    }
    // dump actual version of the libUSB
    libusb_v = libusb_get_version();
    printf("Use libUSB v%d.%d.%d.%d\n", libusb_v->major, libusb_v->minor, libusb_v->micro, libusb_v->nano);
    
    
    printf("List all Yoctopuce devices present on this host:\n");
    
    nbdev = libusb_get_device_list(libusb, &list);
    if (nbdev < 0)
        return ySetErr("Unable to get device list", nbdev);
    
    if (verbose) {
        printf("[%d usb devices present]\n", i);
    }
    
    // allocate buffer for detected interfaces
    nbifaceDetect = 0;
    alloc_size = nbdev  * sizeof(yInterfaceSt);
    ifaces = (yInterfaceSt*) malloc(alloc_size);
    memset(ifaces, 0, alloc_size);
    for (i = 0; i < nbdev; i++) {
        int  res;
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *config;
        libusb_device_handle *hdl;
        
        libusb_device  *dev = list[i];
        if ((res = libusb_get_device_descriptor(dev, &desc)) != 0) {
            returnval = ySetErr("Unable to get device descriptor", res);
            return -1;
        }
        
        if (verbose) {
            printf("[parse device %d = %X:%X]\n", i, desc.idVendor, desc.idProduct);
        }
        
        if (desc.idVendor != YOCTO_VENDORID) {
            continue;
        }
        if(getDevConfig(dev, &config) < 0) {
            continue;
        }
        iface = ifaces + nbifaceDetect;
        iface->vendorid = (unsigned)desc.idVendor;
        iface->deviceid = (unsigned)desc.idProduct;
        iface->ifaceno  = 0;
        iface->devref   = libusb_ref_device(dev);
        res = libusb_open(dev, &hdl);
        if (res == LIBUSB_ERROR_ACCESS) {
            printf("the user has insufficient permissions to access USB devices");
            return -1;
        }
        if (res != 0) {
            printf("unable to access device %X:%X\n", desc.idVendor, desc.idProduct);
            return -1;
        }
        
        res = getUsbStringASCII( hdl, dev, desc.iSerialNumber, iface->serial, YOCTO_SERIAL_LEN);
        if (res < 0) {
            printf("unable to get serial for device %X:%X\n", desc.idVendor, desc.idProduct);
        }
        libusb_close(hdl);
        nbifaceDetect++;
        printf(" - USB dev: %s (%X:%X:%d)\n", iface->serial, iface->vendorid, iface->deviceid, iface->ifaceno);
        libusb_free_config_descriptor(config);
    }
    libusb_free_device_list(list, 1);
    printf("%d Yoctopuce devices are present\n", nbifaceDetect);
    
    for (i = 0; i < nbifaceDetect; i++) {
        int p;
        yInterfaceSt *iface = ifaces + i;
        unsigned char pkt[YOCTO_PKT_SIZE];
        int res, j;
        int error;
        struct libusb_config_descriptor *config;
        const struct libusb_interface_descriptor* ifd;
        
        printf("\\ Test device %s (%X:%X)\n", iface->serial, iface->vendorid, iface->deviceid);
        
        if((res = libusb_open(iface->devref, &iface->hdl)) != 0) {
            return ySetErr("libusb_open", res);
        }
        
        if((res = libusb_kernel_driver_active(iface->hdl, iface->ifaceno)) < 0) {
            return ySetErr("libusb_kernel_driver_active", res);
        }
        if (res) {
            printf("- need to detach kernel driver\n");
            if((res = libusb_detach_kernel_driver(iface->hdl, iface->ifaceno)) < 0) {
                return ySetErr("libusb_detach_kernel_driver", res);
            }
        }
        if((res = libusb_claim_interface(iface->hdl, iface->ifaceno)) < 0) {
            return ySetErr("libusb_claim_interface", res);
        }
        
        res = getDevConfig(iface->devref, &config);
        if(res < 0) {
            return res;
        }
        
        ifd = &config->interface[iface->ifaceno].altsetting[0];
        for (j = 0; j < ifd->bNumEndpoints; j++) {
            if((ifd->endpoint[j].bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
                iface->rdendp = ifd->endpoint[j].bEndpointAddress;
            } else {
                iface->wrendp = ifd->endpoint[j].bEndpointAddress;
            }
        }
        
        iface->rdTr = libusb_alloc_transfer(0);
        res = submitReadPkt(iface);
        if (res < 0) {
            return res;
        }
        
        // send 15 packets
        for (p = 0; p < NB_PKT_TO_TEST; p++) {
            int sleepMs = 1000;
            int transfered;
            struct timeval tv;
            memcpy(pkt, config_pkt, YOCTO_PKT_SIZE);
            printf("+ Send USB pkt no %d\n", iface->sent_pkt);
            res = libusb_interrupt_transfer(iface->hdl,
                                            iface->wrendp,
                                            pkt,
                                            YOCTO_PKT_SIZE,
                                            &transfered,
                                            5000/*5 sec*/);
            if(res < 0 || YOCTO_PKT_SIZE != transfered) {
                printf("USB pkt transmit error %d (transmitted %d / %d)\n", res, transfered, YOCTO_PKT_SIZE);
                break;
            }
            iface->sent_pkt++;
            // wait 1 second
            memset(&tv, 0, sizeof(tv));
            tv.tv_sec = 1;
            res = libusb_handle_events_timeout(libusb, &tv);
            if (res < 0) {
                return ySetErr("libusb_handle_events_timeout", res);
            }
        }
        
        printf("= result: %d pkt sent vs %d pkt received\n",iface->sent_pkt, iface->received_pkt);
        if (iface->sent_pkt != iface->received_pkt) {
            printf("!!! ERROR : %d packets missing !!!\n",iface->sent_pkt - iface->received_pkt);
        }
        printf("Cancel and free all pending transactions\n");
        if (iface->rdTr) {
            int count = 10;
            int res = libusb_cancel_transfer(iface->rdTr);
            if(res == 0) {
                while(count && iface->rdTr->status != LIBUSB_TRANSFER_CANCELLED) {
                    usleep(1000);
                    count--;
                }
            }
        }
        res = libusb_release_interface(iface->hdl, iface->ifaceno);
        if(res != 0 && res != LIBUSB_ERROR_NOT_FOUND && res != LIBUSB_ERROR_NO_DEVICE) {
            printf("%s libusb_release_interface error\n", iface->serial);
        }
        
        if (res < 0 && res != LIBUSB_ERROR_NO_DEVICE) {
            printf("%s libusb_attach_kernel_driver error\n", iface->serial);
        }
        libusb_close(iface->hdl);
        iface->hdl = NULL;
        
        if (iface->rdTr) {
            libusb_free_transfer(iface->rdTr);
            iface->rdTr = NULL;
        }
    }
    
    libusb_exit(libusb);

    
    
    
    return 0;
}
