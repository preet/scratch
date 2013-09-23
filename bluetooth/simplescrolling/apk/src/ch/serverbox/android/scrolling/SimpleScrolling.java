package ch.serverbox.android.scrolling;

import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

public class SimpleScrolling extends Activity {
    /** Called when the activity is first created. */
	
	private WakeLock mLock = null;
	private ImageView mMainView = null;
	private float mStartY = 0;
	private byte byteToSend = 0;
	private BluetoothAdapter mBluetoothAdapter = null;
	public static String sAddress = "00:23:4D:FC:14:20";
	
	public static final int THRESH = 30;
	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater mi = getMenuInflater();
		mi.inflate(R.menu.menu, menu);
		return true;
	}
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		//only one item
		l("onOptionsItemSelected");
		final int SIZE = mBluetoothAdapter.getBondedDevices().size();
		final String[] names = new String[SIZE];
		final String[] addr = new String[SIZE];
		int i = 0;
		for(BluetoothDevice d : mBluetoothAdapter.getBondedDevices()){
			names[i] = d.getName();
			addr[i] = d.getAddress();
			i++;
			if(i >= SIZE)
				break;
		}
		AlertDialog.Builder b = new AlertDialog.Builder(this);
		b.setTitle("Device");
		b.setItems(names, new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				sAddress = addr[which];
				SharedPreferences sp = getSharedPreferences("local", MODE_PRIVATE);
				sp.edit().putString("addr", sAddress).commit();
				restart();
			}
		});
		b.create().show();
		return true;
	}
	
	private void restart(){
    	synchronized (sLock) {
    		mStop = true;
    		sLock.notify();
    		try {
    			while(!mStopped)
    				sLock.wait(500);
			} catch (InterruptedException e) {e(e);}
		}
    	mStop = false;
    	mStopped = false;
    	new Thread(mBTLoop).start();
	}
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        SharedPreferences sp = getSharedPreferences("local", MODE_PRIVATE);
        sAddress =  sp.getString("addr", "00:23:4D:FC:14:20");
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        
        mMainView = (ImageView)findViewById(R.id.imageView1);
        mMainView.setOnTouchListener(new View.OnTouchListener() {
			@Override
			public boolean onTouch(View v, MotionEvent event) {
				if(event.getAction() == MotionEvent.ACTION_MOVE){
					float dy = mStartY - event.getY();
					if(Math.abs(dy) > THRESH){
						mStartY = event.getY();
						queueScroll((byte) (dy > 0 ? 1 : -1));
						synchronized (sLock) {
							sLock.notify();
						}
					}
					
				}else if(event.getAction() == MotionEvent.ACTION_DOWN){
					mStartY = event.getY();
				}
				return true;
			}
		});
        
        
    }
    
    @Override
    protected void onDestroy() {
    	super.onDestroy();
    }
    
    @Override
    protected void onStart() {
    	super.onStart();
    	l("onStart");
        if (!mBluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, 0);
        }
        
    	if(mLock == null){
    		PowerManager pm = ((PowerManager)getSystemService(POWER_SERVICE));
    		mLock = pm.newWakeLock(PowerManager.FULL_WAKE_LOCK, "Simplescrolling");
    	}
    	if(mLock.isHeld() != true)
    		mLock.acquire(); 
    	
    	mStopped = false;
    	new Thread(mBTLoop).start();
    }
    
    @Override
    protected void onStop() {
    	l("onStop");
    	if(mLock != null && mLock.isHeld())
    		mLock.release();
    	
    	synchronized (sLock) {
    		mStop = true;
    		sLock.notify();
    		try {
    			while(!mStopped)
    				sLock.wait(500);
			} catch (InterruptedException e) {e(e);}
		}
    	mStop = false;
    	super.onStop();
    }
    
    private void queueScroll(byte val){
    	byteToSend = val;
    }
    
    
    private final static Object sLock = new Object();
    private boolean mStopped = false;
    private boolean mStop = false;
     
    private Runnable mBTLoop = new Runnable() {
		@Override
		public void run() {
			for(;;){
				synchronized (sLock) {
					if(mStop){
						mStopped = true;
						sLock.notify();
						return;
					}
				}
				
				BluetoothConnection bc = new BluetoothConnection();
				if(bc.initialize(mBluetoothAdapter) < 0){
					try {
						Thread.sleep(1000);
					} catch (InterruptedException e) {e(e);mStopped = true; return;}
					continue;
				}
				
				l("initialized");
				
				for(;;){
					synchronized (sLock) {
						try {
							sLock.wait();//wait for a new event or termination
							if(mStop){								
								//terminate BT
								l("stopping bt connection");
								bc.stop();
								mStopped = true;
								sLock.notify();
								l("terminating thread");
								return;
							}else{//woke up for data phase
								if(bc.sendByte(byteToSend) < 0){
									mStopped = true;//data phase crash, reconnect
									break;
								}
							}
						}catch (InterruptedException e){e(e);}
					}
				}
				try{
					Thread.sleep(2000);
				}catch (InterruptedException e){e(e);}
			}
		}
	};
    
	private void e(Object msg){
		Log.e("ScrollingActivity", ">==< "+msg+" >==<");
	}
	private void l(Object msg){
		Log.d("ScrollingActivity", ">==< "+msg+" >==<");
	}
}