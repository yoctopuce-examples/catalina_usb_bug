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
#include <IOKit/hid/IOHIDLib.h>
#include <pthread.h>

typedef struct {
    IOHIDManagerRef     manager;
    //yCRITICAL_SECTION   hidMCS;
} OSX_HID_REF;



#define YOCTO_ERRMSG_LEN 256
#define YOCTO_SERIAL_LEN 20
#define YOCTO_VENDORID   0x24e0
#define YOCTO_PKT_SIZE   64
#define SETUPED_IFACE_CACHE_SIZE 128


#define YCTX_OSX_MULTIPLES_HID 1

#define NB_PKT_TO_TEST 15




typedef struct _yInterfaceSt {
    unsigned             vendorid;
    unsigned             deviceid;
    unsigned             ifaceno;
    unsigned             pkt_version;
    unsigned             sent_pkt;
    unsigned             received_pkt;
    char            serial[YOCTO_SERIAL_LEN*2];
    int             yyySetupDone;
    //pktQueue        rxQueue;
    //pktQueue        txQueue;
    OSX_HID_REF         hid;
    CFStringRef         run_loop_mode;
    IOHIDDeviceRef      devref;
    //USB_Packet          tmprxpkt;
    unsigned char       incomming_pkt[YOCTO_PKT_SIZE];
} yInterfaceSt;


typedef enum {
    USB_THREAD_NOT_STARTED,
    USB_THREAD_RUNNING,
    USB_THREAD_MUST_STOP,
    USB_THREAD_STOPED
} USB_THREAD_STATE;

typedef struct{
    unsigned int                 osx_flags;
    OSX_HID_REF         hid;
    CFRunLoopRef        usb_run_loop;
    pthread_t           usb_thread;
    yInterfaceSt*       setupedIfaceCache[SETUPED_IFACE_CACHE_SIZE];
    
    USB_THREAD_STATE    usb_thread_state;
    //yCRITICAL_SECTION   parano_cs;
    
} yContextSt;



unsigned char config_pkt[YOCTO_PKT_SIZE] = {
    0x00, 0x15, 0x09, 0x02, 0x01, 0x00, 0x01, 0x00,
    0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

unsigned char response_pkt[YOCTO_PKT_SIZE] = {
    0x00, 0x15, 0x09, 0x02, 0x01, 0x00, 0x01, 0x00,
    0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


int verbose = 0;


static USB_THREAD_STATE get_usb_thread_state(yContextSt  *ctx)
{
    USB_THREAD_STATE res;
    //yEnterCriticalSection(&ctx->parano_cs);
    res =ctx->usb_thread_state;
    //yLeaveCriticalSection(&ctx->parano_cs);
    return res;
}


static void set_usb_thread_state(yContextSt  *ctx, USB_THREAD_STATE st)
{
    //yEnterCriticalSection(&ctx->parano_cs);
    ctx->usb_thread_state = st;
    //yLeaveCriticalSection(&ctx->parano_cs);
}

static void *event_thread(void *param)
{
    yContextSt  *ctx=param;
    
    ctx->usb_run_loop     = CFRunLoopGetCurrent();
    set_usb_thread_state(ctx, USB_THREAD_RUNNING);
    /* Non-blocking. See if the OS has any reports to give. */
    printf("Start event_thread run loop\n");
    while (get_usb_thread_state(ctx) != USB_THREAD_MUST_STOP) {
        CFRunLoopRunInMode( kCFRunLoopDefaultMode, 10, FALSE);
    }
    
    printf("event_thread run loop stopped\n");
    
    set_usb_thread_state(ctx, USB_THREAD_STOPED);
    return NULL;
}




static int setupHIDManager(yContextSt *ctx, OSX_HID_REF *hid)
{
    int             c_vendorid = YOCTO_VENDORID;
    CFMutableDictionaryRef dictionary;
    CFNumberRef     Vendorid;
    IOReturn        tIOReturn;

    //yInitializeCriticalSection(&hid->hidMCS);
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
        printf("Unable to Open HID Manager\n");
        return -1;
    }
    return 0;

}




static void stopHIDManager(OSX_HID_REF *hid)
{
    if (hid->manager) {
        IOHIDManagerClose(hid->manager, kIOHIDOptionsTypeNone );
        CFRelease( hid->manager);
        hid->manager=NULL;
        //yDeleteCriticalSection(&hid->hidMCS);
    }
}

int yyyUSB_init(yContextSt *ctx)
{
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
        printf("kern.osrelease=%s\n", str);

        str[2]=0;
        numver = atoi(str);
        if (numver >= 13 && numver < 15){
            ctx->osx_flags |= YCTX_OSX_MULTIPLES_HID;
        }
    }

    //yInitializeCriticalSection(&ctx->parano_cs);
    set_usb_thread_state(ctx, USB_THREAD_NOT_STARTED);
    pthread_create(&ctx->usb_thread, NULL, event_thread, ctx);
    while(get_usb_thread_state(ctx) != USB_THREAD_RUNNING){
        usleep(50000);
    }

    if (setupHIDManager(ctx, &ctx->hid)<0) {
        printf(" - ERROR: setupHIDManager\n");
        return -1;
    }
    return 0;
}


int yyyUSB_stop(yContextSt *ctx)
{
    stopHIDManager(&ctx->hid);
    
    if(get_usb_thread_state(ctx) == USB_THREAD_RUNNING){
        set_usb_thread_state(ctx, USB_THREAD_MUST_STOP);
        CFRunLoopStop(ctx->usb_run_loop);
    }
    pthread_join(ctx->usb_thread,NULL);
    if(get_usb_thread_state(ctx) != USB_THREAD_STOPED){
        printf("USB thread did not stop correctly!\n");
    }
    
    return 0;
}



static unsigned get_int_property(IOHIDDeviceRef device, CFStringRef key)
{
    CFTypeRef ref;
    unsigned value;
    
    ref = IOHIDDeviceGetProperty(device, key);
    if (ref) {
        if (CFGetTypeID(ref) == CFNumberGetTypeID() && CFNumberGetValue((CFNumberRef) ref, kCFNumberSInt32Type, &value)) {
            return value;
        }
    }
    return 0;
}


static void get_txt_property(IOHIDDeviceRef device,char *buffer,unsigned maxlen, CFStringRef key)
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
                char *tmp_str = malloc(cflength);
                if (!CFStringGetCString(ref, tmp_str, cflength, kCFStringEncodingUTF8 )) {
                    free( tmp_str );
                    *buffer=0;
                    return;
                }
                if(cflength>maxlen-1){
                    cflength=maxlen-1;
                }
                memcpy(buffer,tmp_str,cflength);
                buffer[cflength]=0;
                free( tmp_str );
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
    
    //yEnterCriticalSection(&hid->hidMCS);
    deviceCFSetRef = IOHIDManagerCopyDevices(hid->manager);
    //yLeaveCriticalSection(&hid->hidMCS);
    if (deviceCFSetRef!= NULL) {
        // how many devices in the set?
        *deviceCount = CFSetGetCount( deviceCFSetRef );
        dev_refs = malloc( sizeof(IOHIDDeviceRef) * (int)*deviceCount );
        // now extract the device ref's from the set
        CFSetGetValues( deviceCFSetRef, (const void **) dev_refs );
    }
    return dev_refs;
}




int yyyUSBGetInterfaces(yContextSt *ctx, yInterfaceSt **ifaces,int *nbifaceDetect)
{
    int             nbifaceAlloc;
    int             deviceIndex;
    CFIndex         deviceCount;
    IOHIDDeviceRef  *dev_refs;

    // get all device detected by the OSX
    dev_refs = getDevRef(&ctx->hid, &deviceCount);
    if(dev_refs == NULL) {
        return 0;
    }

    // allocate buffer for detected interfaces
    *nbifaceDetect = 0;
    nbifaceAlloc  = 8;
    *ifaces =malloc(nbifaceAlloc * sizeof(yInterfaceSt));
    memset(*ifaces, 0 ,nbifaceAlloc * sizeof(yInterfaceSt));
    for(deviceIndex=0 ; deviceIndex < deviceCount ;deviceIndex++){
        unsigned int vendorid;
        unsigned int deviceid;
        IOHIDDeviceRef dev = dev_refs[deviceIndex];
        yInterfaceSt    *iface;
        vendorid = get_int_property(dev,CFSTR(kIOHIDVendorIDKey));
        deviceid = get_int_property(dev,CFSTR(kIOHIDProductIDKey));
        //ensure the buffer of detected interface is big enought
        if(*nbifaceDetect == nbifaceAlloc){
            yInterfaceSt    *tmp;
            tmp = (yInterfaceSt*) malloc(nbifaceAlloc*2 * sizeof(yInterfaceSt));
            memset(tmp,0,nbifaceAlloc*2 * sizeof(yInterfaceSt));
            memcpy(tmp,*ifaces, nbifaceAlloc * sizeof(yInterfaceSt) );
            free(*ifaces);
            *ifaces = tmp;
            nbifaceAlloc    *=2;
        }
        iface = *ifaces + *nbifaceDetect;
        //iface->devref   = dev;
        iface->vendorid = vendorid;
        iface->deviceid = deviceid;
        get_txt_property(dev,iface->serial,YOCTO_SERIAL_LEN*2, CFSTR(kIOHIDSerialNumberKey));
        printf("work on interface %d (%x:%x:%s)\n",deviceIndex,vendorid,deviceid,iface->serial);
        (*nbifaceDetect)++;
    }
    free(dev_refs);
    return 0;
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
    //yPktQueuePushD2H(iface,&iface->tmprxpkt,NULL);
    if ( InReportLength == YOCTO_PKT_SIZE && memcmp(iface->incomming_pkt, response_pkt,7)==0) {
        printf("- rd_callback for %s : response %d is valid (len=%d)\n", iface->serial, iface->received_pkt, (int)InReportLength);
        iface->received_pkt++;
    } else {
        int i,j;
        printf("- rd_callback for %s : response  %d is INVAID! (len=%d)\n", iface->serial, iface->received_pkt, (int)InReportLength);
        printf("Incoming pkt:\n");
        for(i = 0; i < 8; i++) {
            for(j = 0; j < 7; j++) {
                printf("0x%02x ",iface->incomming_pkt[i * 8 + j]);
            }
            printf("0x%02x\n",iface->incomming_pkt[i * 8 + 7]);
            
        }
    }
    memset(&iface->incomming_pkt,0xff,YOCTO_PKT_SIZE);
}


int yyySetup(yContextSt *ctx, yInterfaceSt *iface)
{
    char str[32];
    int i;
    CFIndex deviceCount;
    IOHIDDeviceRef *dev_refs;
    
    
    if (ctx->osx_flags & YCTX_OSX_MULTIPLES_HID) {
        if (setupHIDManager(ctx, &iface->hid)<0) {
            return -1;
        }
        // get all device detected by the OSX
        dev_refs = getDevRef(&iface->hid, &deviceCount);
    } else {
        dev_refs = getDevRef(&ctx->hid, &deviceCount);
    }
    if(dev_refs == NULL) {
        printf("Device disappear before yyySetup\n");
        return -1;
    }
    
    
    for(i=0 ; i < deviceCount ;i++){
        unsigned vendorid;
        unsigned deviceid;
        IOHIDDeviceRef dev = dev_refs[i];
        vendorid = get_int_property(dev,CFSTR(kIOHIDVendorIDKey));
        deviceid = get_int_property(dev,CFSTR(kIOHIDProductIDKey));
        if (iface->vendorid == vendorid && iface->deviceid == deviceid){
            char serial[YOCTO_SERIAL_LEN * 2];
            memset(serial, 0, YOCTO_SERIAL_LEN * 2);
            get_txt_property(dev,serial,YOCTO_SERIAL_LEN * 2, CFSTR(kIOHIDSerialNumberKey));
            if (strcmp(serial, iface->serial) == 0){
                printf("right Interface detected (%x:%x:%s)\n",vendorid,deviceid,iface->serial);
                iface->devref = dev;
                break;
            }
        }
    }
    free(dev_refs);
    if (i == deviceCount) {
        printf("Unable to match device detected\n");
        return -1;
    }
    
    IOReturn ret = IOHIDDeviceOpen(iface->devref, kIOHIDOptionsTypeNone);
    if (ret != kIOReturnSuccess) {
        printf("Unable to open device (0x%x)\n",ret);
        return -1;
    }
    
    
    /* Create the Run Loop Mode for this device. printing the reference seems to work. */
    sprintf(str, "yocto_%p", iface->devref);
    iface->run_loop_mode = CFStringCreateWithCString(NULL, str, kCFStringEncodingASCII);
    /* Attach the device to a Run Loop */
    IOHIDDeviceScheduleWithRunLoop(iface->devref, ctx->usb_run_loop, iface->run_loop_mode);
    IOHIDDeviceRegisterInputReportCallback( iface->devref,              // IOHIDDeviceRef for the HID device
                                           iface->incomming_pkt,     // pointer to the report data
                                           YOCTO_PKT_SIZE,               // number of bytes in the report (CFIndex)
                                           &Handle_IOHIDDeviceIOHIDReportCallback,   // the callback routine
                                           iface);                     // context passed to callback
    
    // save setup ed iface pointer in context in order
    // to retrieve it during unplugcallback
    for (i=0; i< SETUPED_IFACE_CACHE_SIZE ; i++) {
        if(ctx->setupedIfaceCache[i]==NULL){
            ctx->setupedIfaceCache[i] = iface;
            break;
        }
    }
    if (i==SETUPED_IFACE_CACHE_SIZE) {
        printf("Too many setup USB interfaces");
        return -1;
    }
    iface->yyySetupDone = 1;
    return 0;
}


void yyyPacketShutdown(yContextSt *ctx, yInterfaceSt  *iface)
{
    int i;
    
    // remove iface from setup ifaces
    for (i=0; i< SETUPED_IFACE_CACHE_SIZE ; i++) {
        if(ctx->setupedIfaceCache[i]==iface){
            ctx->setupedIfaceCache[i] = NULL;
            break;
        }
    }
    if(i>=SETUPED_IFACE_CACHE_SIZE){
        printf("error in setuped cache\n");
    }
    if(iface->devref!=NULL){
        IOHIDDeviceRegisterInputReportCallback(iface->devref,              // IOHIDDeviceRef for the HID device
                                               iface->incomming_pkt,   // pointer to the report data (uint8_t's)
                                               YOCTO_PKT_SIZE,              // number of bytes in the report (CFIndex)
                                               NULL,   // the callback routine
                                               iface);                      // context passed to callback
        IOHIDDeviceClose(iface->devref, kIOHIDOptionsTypeNone);
        iface->devref=NULL;
    }
    iface->yyySetupDone = 0;
    CFRelease(iface->run_loop_mode);
    if (ctx->osx_flags & YCTX_OSX_MULTIPLES_HID) {
        stopHIDManager(&iface->hid);
    }
}


int main(int argc, const char * argv[]) {
    int nbifaceDetect, i;
    yContextSt      yctx;
    yInterfaceSt    *ifaces;
   

    printf("This is a simple program that exhibit a bug in the USB stackof macOS Catalina\n");
    printf("You need to have any Yoctopuce device pluged on USB\n");

    if (argc > 1 && argv[1]){
        verbose = 1;
    }

    if (yyyUSB_init(&yctx) !=0){
        printf("- Error in yyyUSB_init\n");
        return 1;
    }


    printf("List all Yoctopuce devices present on this host:\n");

    if (yyyUSBGetInterfaces(&yctx, &ifaces,&nbifaceDetect)!=0){
        printf("- Error in yyyUSBGetInterfaces:\n");
    }

    printf("%d Yoctopuce devices are present\n", nbifaceDetect);

    for (i = 0; i < nbifaceDetect; i++) {
        int p;
        yInterfaceSt *iface = ifaces + i;
        unsigned char pkt[YOCTO_PKT_SIZE];
       
        printf("\\ Test device %s (%X:%X)\n", iface->serial, iface->vendorid, iface->deviceid);

        if(yyySetup(&yctx,iface)!=0) {
            printf("- Error in yyySetup\n");
            return 1;
        }

        // send 15 packets
        for (p = 0; p < NB_PKT_TO_TEST; p++) {
            memcpy(pkt, config_pkt, YOCTO_PKT_SIZE);
            printf("+ Send USB pkt no %d\n", iface->sent_pkt);
            int res = IOHIDDeviceSetReport(iface->devref,
                                           kIOHIDReportTypeOutput,
                                           0, /* Report ID*/
                                           pkt, YOCTO_PKT_SIZE);
            if (res != kIOReturnSuccess) {
                printf("IOHIDDeviceSetReport failed with 0x%x\n", res);
                return -1;
            }
            iface->sent_pkt++;
            // wait 1 second
            sleep(1);
        }
        printf("= result: %d pkt sent vs %d pkt received\n",iface->sent_pkt, iface->received_pkt);
        if (iface->sent_pkt != iface->received_pkt) {
            printf("!!! ERROR : %d packets missing !!!\n",iface->sent_pkt - iface->received_pkt);
        }
        printf("Cancel and free all pending transactions\n");
        yyyPacketShutdown(&yctx, iface);
    }

    if (yyyUSB_stop(&yctx) !=0){
        printf("- Error in yyyUSB_stop\n");
        return 1;
    }

    return 0;
}
