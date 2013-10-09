package ca.predesign.prismatic;

import android.util.Log;
import android.os.Bundle;
import android.content.Context;
import android.content.Intent;
import android.content.BroadcastReceiver;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

import java.lang.String;
import java.util.Set;

public class PrisBluetoothStatusReceiver extends BroadcastReceiver
{
//---------------------------------------------------------------------------

   // status may be true (ON) or false (OFF)
   public native void jni_onBluetoothAdapterStatusChanged(boolean status);

   // called when a remote device is found
   public native void jni_onBluetoothDeviceFound(BluetoothDevice device,
                                                 String deviceName,
                                                 String deviceAddress);

   // called when a connection is established
   // with a remote device
   public native void jni_onBluetoothDeviceConnected(BluetoothDevice device,
                                                     String deviceName,
                                                     String deviceAddress);

   // called when a remote device is disconnected
   public native void jni_onBluetoothDeviceDisconnected(BluetoothDevice device,
                                                        String deviceName,
                                                        String deviceAddress);

   @Override
   public void onReceive(Context context, Intent intent)   {
      // debug
      //Log.i("PrisBluetoothStatusReceiver","####: onReceive called\n\n\n");

      String action = intent.getAction();

      if(action.equals(BluetoothAdapter.ACTION_STATE_CHANGED))   {
         // Indicates the state of the bluetooth adapter
         // on the device has changed

         // extras should contain the following keys:
         // BluetoothAdapter.EXTRA_STATE
         // BluetoothAdapter.EXTRA_PREVIOUS_STATE

         // where the values are one of the integers:
         // BluetoothAdapter.STATE_OFF
         // BluetoothAdapter.STATE_ON
         // BluetoothAdapter.STATE_TURNING_OFF
         // BluetoothAdapter.STATE_TURNING_ON
         Bundle extras = intent.getExtras();

         if(extras != null)   {
            int status = extras.getInt(BluetoothAdapter.EXTRA_STATE);
            if(status == BluetoothAdapter.STATE_ON)   {
               jni_onBluetoothAdapterStatusChanged(true);
            }
            else if(status == BluetoothAdapter.STATE_OFF)   {
               jni_onBluetoothAdapterStatusChanged(false);
            }
         }
      }

      else if(action.equals(BluetoothDevice.ACTION_FOUND))   {
         // Indicates the bluetooth adapter has discovered
         // a remote device after a scan has been performed
         BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
         String deviceAddress = device.getAddress();
         String deviceName = device.getName();
         if(deviceName == null)   {
            deviceName = "";
         }
         jni_onBluetoothDeviceFound(device,deviceName,deviceAddress);
      }

      else if(action.equals(BluetoothDevice.ACTION_ACL_CONNECTED))   {
         // Indicates that a low level ACL connection
         // has been established with a remote device
         BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
         String deviceAddress = device.getAddress();
         String deviceName = device.getName();
         if(deviceName == null)   {
            deviceName = "";
         }
         jni_onBluetoothDeviceConnected(device,deviceName,deviceAddress);
      }

      else if(action.equals(BluetoothDevice.ACTION_ACL_DISCONNECTED))   {
         // Indicates that a low level ACL connection
         // with a remote device has been terminated
         BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
         String deviceAddress = device.getAddress();
         String deviceName = device.getName();
         if(deviceName == null)   {
            deviceName = "";
         }
         jni_onBluetoothDeviceDisconnected(device,deviceName,deviceAddress);
      }
   }

//---------------------------------------------------------------------------
}
