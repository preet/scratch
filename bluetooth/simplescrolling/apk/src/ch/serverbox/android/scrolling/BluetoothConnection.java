package ch.serverbox.android.scrolling;

import java.io.IOException;
import java.util.UUID;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

public class BluetoothConnection {
	
	BluetoothSocket mSocket = null;
	//0x4e65,0x7875,0x732d,0x436f,0x6d70,0x7574,0x696e,0x6700
	//private final UUID APPUUID = UUID.fromString("4e657875-732d-436f-6d70-7574696e6700");
	//private final UUID APPUUID = UUID.fromString("654e0000-7578-0000-2d73-00006f430000");
	//0x07,0x29,0x3d,0xb4,0xa3, 0x23, 0x4e, 0x07,0x8b, 0x8b,0x25,0x0b,0x34,0x0e, 0x42, 0xa4
	private final UUID APPUUID = UUID.fromString("07293db4-a323-4e07-8b8b-250b340e42a4");
	public int initialize(BluetoothAdapter a){ 
		try {
			mSocket = a.getRemoteDevice(SimpleScrolling.sAddress).createRfcommSocketToServiceRecord(APPUUID);
		} catch (IOException e) {e(e);return -1;}
		l("socket created");
		try {
			mSocket.connect();
		} catch (IOException e) {e(e);return -1;}
		l("connected"); 
		return 0;
	}
	
	public int sendByte(byte b){
		l("trying to send "+String.format("%02X", b));
		try { 
			mSocket.getOutputStream().write(b);
		} catch (IOException e) {e(e);return -1;}
		return 0;
	}
	
	public int stop(){
		if(mSocket != null){
			try {
				mSocket.close();
			} catch (IOException e) {e(e);return -1;}
		}
		
		return 0;
	}
	private void e(Object msg){
		Log.e("BluetoothConnection", ">==< "+msg+" >==<");
	}
	private void l(Object msg){
		Log.d("BluetoothConnection", ">==< "+msg+" >==<");
	}
}
