// http://www.freedesktop.org/software/systemd/libudev/
// http://www.signal11.us/oss/udev/
// udev-notify (see list of devices at btm of this file)

#include <libudev.h>
#include <QDebug>
#include <QString>

/*
int main(void)
{
    struct udev *udev;
    struct udev_device *dev;

    // create udev object
    udev = udev_new();
    if (!udev) {
        printf("Can't create udev\n");
        exit(1);
    }

    dev = udev_device_new_from_subsystem_sysname(udev,"block","sdb1");

    // print information
    qDebug() << "==================================================";
    qDebug() << "Device Path:"      << udev_device_get_devpath(dev);
    qDebug() << "Device Node Path:" << udev_device_get_devnode(dev);
    qDebug() << "Device Subsystem:" << udev_device_get_subsystem(dev);
    qDebug() << "Device Type:"      << udev_device_get_devtype(dev);
    qDebug() << "Device Sys Path:"  << udev_device_get_syspath(dev);
    qDebug() << "Device Sys Name:"  << udev_device_get_sysname(dev);
    qDebug() << "Device Sys Num:"   << udev_device_get_sysnum(dev);

    // print property list
    struct udev_list_entry *list_properties,*property_list_entry;
    list_properties = udev_device_get_properties_list_entry(dev);
    udev_list_entry_foreach(property_list_entry,list_properties)   {
        qDebug() << "-> Property Name:" << udev_list_entry_get_name(property_list_entry)
                 << ", Value:" << udev_list_entry_get_value(property_list_entry);
    }

    return 0;
}
*/


int main(void)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *list_devices,*dev_list_entry;
    struct udev_device *dev;

    // create udev object
    udev = udev_new();
    if (!udev) {
        printf("Can't create udev\n");
        exit(1);
    }

    // list mice on system
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate,"tty");
    udev_enumerate_scan_devices(enumerate);
    list_devices = udev_enumerate_get_list_entry(enumerate);

    // iterate through list
    udev_list_entry_foreach(dev_list_entry,list_devices)   {

        // get device path in /sys
        const char * path;
        path = udev_list_entry_get_name(dev_list_entry);

        // get device
        dev = udev_device_new_from_syspath(udev,path);

        // print information
        qDebug() << "==================================================";
        qDebug() << "Device Path:"      << udev_device_get_devpath(dev);
        qDebug() << "Device Node Path:" << udev_device_get_devnode(dev);
        qDebug() << "Device Subsystem:" << udev_device_get_subsystem(dev);
        qDebug() << "Device Type:"      << udev_device_get_devtype(dev);
        qDebug() << "Device Sys Path:"  << udev_device_get_syspath(dev);
        qDebug() << "Device Sys Name:"  << udev_device_get_sysname(dev);
        qDebug() << "Device Sys Num:"   << udev_device_get_sysnum(dev);

        // print property list
        struct udev_list_entry *list_properties,*property_list_entry;
        list_properties = udev_device_get_properties_list_entry(dev);
        udev_list_entry_foreach(property_list_entry,list_properties)   {
            qDebug() << "-> Property Name:" << udev_list_entry_get_name(property_list_entry)
                     << ", Value:" << udev_list_entry_get_value(property_list_entry);
        }
    }

    return 0;
}


//devices = [
//    {
//        'type': _('CD-ROM Drive'),
//        'icon': 'media-optical',
//        'detection': {
//            'DEVTYPE': 'disk',
//            'ID_TYPE': 'cd',
//            'SUBSYSTEM': 'block'
//        }
//    },
//    {
//        'type': _('Multimedia Player'), #this must be BEFORE generic storage and USB devices
//        'icon': 'multimedia-player',
//        'detection': {
//            'DEVTYPE':'usb_device',
//            'SUBSYSTEM': 'usb',
//            'ID_MEDIA_PLAYER': '*',
//        }
//    },
//    #{
//    #    'type': _('Disk Partition'),
//    #    'icon': 'drive-removable-media',
//    #    'detection': {
//    #        'DEVTYPE':'partition',
//    #        'ID_FS_USAGE':'filesystem',
//    #        'ID_TYPE':'disk',
//    #        'SUBSYSTEM':'block'
//    #    }
//    #},
//    {
//        'type': _('USB Storage Device'), # MemoryStick Reader?
//        'icon': 'gnome-dev-media-ms',
//        'detection': {
//            'DEVTYPE':'disk',
//            'ID_TYPE':'disk',
//            'SUBSYSTEM':'block',
//            'ID_BUS': 'usb',
//            'ID_DRIVE_FLASH_MS' : '1'
//        }
//    },
//    {
//        'type': _('USB Storage Device'), #SmartMedia Reader?
//        'icon': 'gnome-dev-media-sm',
//        'detection': {
//            'DEVTYPE':'disk',
//            'ID_TYPE':'disk',
//            'SUBSYSTEM':'block',
//            'ID_BUS': 'usb',
//            'ID_DRIVE_FLASH_SM' : '1'
//        }
//    },
//    {
//        'type': _('USB Storage Device'), #CompatFlash Reader?
//        'icon': 'gnome-dev-media-cf',
//        'detection': {
//            'DEVTYPE':'disk',
//            'ID_TYPE':'disk',
//            'SUBSYSTEM':'block',
//            'ID_BUS': 'usb',
//            'ID_DRIVE_FLASH_CF' : '1'
//        }
//    },
//    {
//        'type': _('SD/MMC Memory'),
//        'icon': 'gnome-dev-media-sdmmc',
//        'detection': {
//            'SUBSYSTEM':'mmc'
//        }
//    },
//    {
//        'type': _('Memory Stick'),
//        'icon': 'gnome-dev-media-ms',
//        'detection': {
//            'SUBSYSTEM':'memstick'
//        }
//    },
//    {
//        'type': _('USB Storage Device'), #SD Card Reader?
//        'icon': 'gnome-dev-media-sdmmc',
//        'detection': {
//            'DEVTYPE':'disk',
//            'ID_TYPE':'disk',
//            'SUBSYSTEM':'block',
//            'ID_BUS': 'usb',
//            'ID_DRIVE_FLASH_SD' : '1'
//        }
//    },
//    {
//        'type': _('USB Storage Device'),
//        'icon': 'drive-removable-media',
//        'detection': {
//            'DEVTYPE':'disk',
//            'ID_TYPE':'disk',
//            'SUBSYSTEM':'block',
//            'ID_BUS': 'usb'
//        }
//    },
//    {
//        'type': _('WiFi Device'),
//        'icon': 'network-wireless',
//        'detection': {
//            'DEVTYPE':'wlan',
//            'SUBSYSTEM': 'net'
//        }
//    },
//    {
//        'type': _('WebCam / TV Tuner'),
//        'icon': 'camera-web',
//        'detection': {
//            'SUBSYSTEM': 'video4linux'
//        }
//    },
//    {
//        'type': _('Mouse'),
//        'icon': 'mouse',
//        'detection': {
//            'ID_INPUT_MOUSE': '1',
//            'ID_TYPE': 'hid',
//            'SUBSYSTEM': 'input'
//        }
//    },
//    {
//        'type': _('Game Controller'),
//        'icon': 'joystick',
//        'detection': {
//            'ID_INPUT_JOYSTICK': '1',
//            'ID_TYPE': 'hid',
//            'SUBSYSTEM': 'input'
//        }
//    },
//    {
//        'type': _('Sound Card'), # needs testing
//        'icon': 'sound',
//        'detection': {
//            'ID_TYPE': 'sound',
//            'SUBSYSTEM': 'sound'
//        }
//    },

//    {
//        'type': _('USB Modem'),
//        'icon': 'modem',
//        'detection': {
//            'ID_BUS':'usb',
//            'SUBSYSTEM': 'tty',
//        }
//    },
//    {
//        'type': _('Modem'),
//        'icon': 'modem',
//        'detection': {
//            'ID_USB_DRIVER': 'cdc_acm',
//            'SUBSYSTEM': 'tty'
//        }
//    },
//    {
//        'type': _('PDA Device'),
//        'icon': 'pda',
//        'detection': {
//            'ID_USB_DRIVER': 'ipaq',
//            'SUBSYSTEM': 'tty'
//        }
//    },
//    {
//        'type': _('Keyboard'),
//        'icon': 'gnome-dev-keyboard',
//        'detection': {
//            'ID_CLASS': 'kbd',
//            'ID_TYPE': 'hid',
//            'SUBSYSTEM': 'input'
//        }
//    },
//    {
//        'type': _('Digital Camera'),
//        'icon': 'camera-photo',
//        'detection': {
//            'DEVTYPE':'usb_device',
//            'ID_GPHOTO2': '1',
//        }
//    },
//    {
//        'type': _('Network Device'),
//        'icon': 'network-wired',
//        'detection': {
//            'ID_BUS':'pci',
//            'SUBSYSTEM': 'net',
//        }
//    },
//    {
//        'type': _('USB Network Device'),
//        'icon': 'network-wired',
//        'detection': {
//            'ID_BUS':'usb',
//            'SUBSYSTEM': 'net',
//        }
//    },
//    {
//        'type': _('USB Device'),
//        'icon': 'gnome-dev-unknown-usb',
//        'detection': {
//            'DEVTYPE':'usb_device',
//            'SUBSYSTEM': 'usb',
//        }
//    },
//]
