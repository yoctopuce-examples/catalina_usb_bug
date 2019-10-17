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





#define YOCTO_ERRMSG_LEN 256
#define YOCTO_SERIAL_LEN 20
#define YOCTO_VENDORID   0x24e0
#define YOCTO_PKT_SIZE   64


#define NB_PKT_TO_TEST 15

typedef struct{
    u32                 osx_flags;
    OSX_HID_REF         hid;
    CFRunLoopRef        usb_run_loop;
    pthread_t           usb_thread;
    USB_THREAD_STATE    usb_thread_state;
    yCRITICAL_SECTION   parano_cs;

 } yContextSt;

typedef struct _yInterfaceSt {
    u16             vendorid;
    u16             deviceid;
    u16             ifaceno;
    u16             pkt_version;
    char            serial[YOCTO_SERIAL_LEN*2];
    int             yyySetupDone;
    pktQueue        rxQueue;
    pktQueue        txQueue;
    OSX_HID_REF         hid;
    CFStringRef         run_loop_mode;
    IOHIDDeviceRef      devref;
    USB_Packet          tmprxpkt;
} yInterfaceSt;

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



int yyyUSB_init(yContextSt *ctx,char *errmsg)
{
    char str[256];
    size_t size = sizeof(str);
    YPROPERR(yReserveGlobalAccess(ctx, errmsg));

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
    return YAPI_SUCCESS;
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


int main(int argc, const char * argv[]) {
    int res;
    int nbifaceDetect, i;
    ssize_t         nbdev;
    int             returnval = 0;
    unsigned             alloc_size;
    yContextSt      yctx;
    yInterfaceSt    *iface, *ifaces;
    char            errmsg[YOCTO_ERRMSG_LEN];


    printf("this is a simple program that exhibit a bug in the USB stack\n");
    printf("of macOS Catalina and any Yoctopuce device\n");

    if (argc > 1 && argv[1]){
        verbose = 1;
    }

    if (yyyUSB_init(&yctx, errmsg) !=0){
        printf("- Error in yyyUSB_init:%s\n",errmsg);
        return 1;
    }


    printf("List all Yoctopuce devices present on this host:\n");

    if (yyyUSBGetInterfaces(&ifaces,&nbifaceDetect,errmsg)!=0){
        printf("- Error in yyyUSBGetInterfaces:%s\n",errmsg);
    }

    printf("%d Yoctopuce devices are present\n", nbifaceDetect);

    for (i = 0; i < nbifaceDetect; i++) {
        int p;
        yInterfaceSt *iface = ifaces + i;
        unsigned char pkt[YOCTO_PKT_SIZE];
        int res, j;
        int error;

        printf("\\ Test device %s (%X:%X)\n", iface->serial, iface->vendorid, iface->deviceid);

        if(yyySetup(iface,errmsg)!=0) {
            printf("- Error in yyySetup:%s\n",errmsg);
            return 1;
        }

        // send 15 packets
        for (p = 0; p < NB_PKT_TO_TEST; p++) {
            int sleepMs = 1000;
            int transfered;
            struct timeval tv;
            memcpy(pkt, config_pkt, YOCTO_PKT_SIZE);
            printf("+ Send USB pkt no %d\n", iface->sent_pkt);




             if (yyySendPacket(iface,pkt, errmsg) != 0){
                printf("USB pkt transmit error %d (transmitted %d / %d)\n", res, transfered, YOCTO_PKT_SIZE);
                printf("- Error in yyySetup:%s\n",errmsg);
                return 1;
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

        yyyPacketShutdown(iface);
    }

    if (yyyUSB_stop(&yctx, errmsg) !=0){
        printf("- Error in yyyUSB_stop:%s\n",errmsg);
        return 1;
    }

    return 0;
}
