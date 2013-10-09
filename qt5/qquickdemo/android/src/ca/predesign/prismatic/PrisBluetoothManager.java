package ca.predesign.prismatic;

import android.util.Log;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;

public class PrisBluetoothManager
{
//---------------------------------------------------------------------------

    private boolean m_init;
    private boolean m_bluetoothAvailable;
    private BluetoothAdapter m_bluetoothAdapter;

    public PrisBluetoothManager()
    {
       m_bluetoothAvailable = false;

       // Get this device's bluetooth Adapter
       m_bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
       if(m_bluetoothAdapter == null)   {
          // device does not support bluetooth
          return;
       }
       m_bluetoothAvailable = true;

       m_init = true;
    }

    public boolean GetBluetoothAvailable()
    {
       return m_bluetoothAvailable;
    }

    public boolean GetBluetoothEnabled()
    {
       return m_bluetoothAdapter.isEnabled();
    }

    public BluetoothAdapter GetAdapter()
    {
       return m_bluetoothAdapter;
    }

//    public Set<BluetoothDevice> GetPairedDevices()
//    {
//       return m_bluetoothAdapter.getBondedDevices();
//    }




//---------------------------------------------------------------------------
}
