package ca.predesign.prismatic;

import android.util.Log;
import android.os.Bundle;
import android.content.Context;
import android.content.Intent;
import android.content.BroadcastReceiver;
import android.bluetooth.BluetoothAdapter;

import java.util.Set;

public class PrisBluetoothStatusReceiver extends BroadcastReceiver
{
//---------------------------------------------------------------------------

   // status may be true (ON) or false (OFF)
   public native void jni_onBluetoothAdapterStatusChanged(boolean status);

   @Override
   public void onReceive(Context context, Intent intent)   {
      // debug
      //Log.i("PrisBluetoothStatusReceiver","####: onReceive called\n\n\n");

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

//---------------------------------------------------------------------------
}
