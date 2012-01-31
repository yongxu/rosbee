package ros.bee;

import ros.Acellerometer.AccelerometerListener;
import ros.Acellerometer.AccelerometerManager;
import ros.UDP.UDPClient;
import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.Menu;
import android.view.MenuInflater;
import android.widget.ImageView;
import android.widget.ImageView.ScaleType;
import android.widget.TextView;

public class RosbeeRemoteActivity extends Activity  implements AccelerometerListener {
	
	private UDPClient _client;
	private static Context CONTEXT;
	private RecvThread _RecvThread;
	private Thread _imgRecvThread;
	private Img_RecvThread rt;
	ImageView imgView;
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        CONTEXT = this;        
        _client = UDPClient.GetInstance("192.168.1.185",1234); 
        
        _RecvThread = new RecvThread(_client.getSeverSock(),((TextView) findViewById(R.id.errorTextField)));
		_RecvThread.start();
		
		imgView = (ImageView)findViewById(R.id.imgView);
		imgView.setScaleType(ScaleType.FIT_XY);
		
		rt = new Img_RecvThread("192.168.1.151", 1337, imgView, new Handler(){
			
			@Override
			public void dispatchMessage(Message msg) {
				RosbeeRemoteActivity.this.UpdateImage(rt.GetImage());
				super.dispatchMessage(msg);
			}
			
		});
		
		_imgRecvThread = new Thread(rt);
		_imgRecvThread.start();
		
      
    }
    
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
	    MenuInflater inflater = getMenuInflater();
	    inflater.inflate(R.layout.optionmenu, menu);
	    return true;
	}

    public static Context getContext() {
                return CONTEXT;
        }
    
	@Override
	 protected void onResume() {
	    	super.onResume();
	    	if (AccelerometerManager.isSupported()) {
	    		AccelerometerManager.startListening(this);
	    	}
	    }
	
	@Override 
	    protected void onDestroy() {
	    	super.onDestroy();
	    	if (AccelerometerManager.isListening()) {
	    		AccelerometerManager.stopListening();
	    	}	    	
	    }
	    
	public void onAccelerationChanged(float x, float y, float z) {
	
		_client.sendUDPString(x+";"+y+";"+z);
		/*((TextView) findViewById(R.id.x)).setText(String.valueOf(x));
		((TextView) findViewById(R.id.y)).setText(String.valueOf(y));
		((TextView) findViewById(R.id.z)).setText(String.valueOf(z));*/		
	}
	
	private void UpdateImage(Bitmap bmp)
	{
		imgView.setImageBitmap(bmp);
	}
	

}