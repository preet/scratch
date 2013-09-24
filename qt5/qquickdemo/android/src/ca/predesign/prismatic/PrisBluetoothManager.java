package ca.predesign.prismatic;

import android.util.Log;
import android.bluetooth.BluetoothAdapter;

public class PrisBluetoothManager
{
//---------------------------------------------------------------------------

    private boolean m_init;
    private boolean m_bluetoothAvailable;
    private BluetoothAdapter m_bluetoothAdapter;

    public PrisBluetoothManager()
    {
       m_init = false;
       m_bluetoothAvailable = false;
    }

    public void Initialize()
    {
       if(m_init)   {
          Log.i("PrisBluetoothManager","WARN: Reinit attempt!");
          return;
       }

       // Get the Bluetooth Adapter
       m_bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
       if(m_bluetoothAdapter == null)   {
          // device does not support bluetooth
          return;
       }
       m_bluetoothAvailable = true;

       m_init = true;
    }

    public BluetoothAdapter GetDeviceAdapter()
    {
       return m_bluetoothAdapter;
    }

    public boolean CheckIfAdapterEnabled()
    {
       return m_bluetoothAdapter.isEnabled();
    }
//---------------------------------------------------------------------------
}
