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
import android.view.MenuItem;
import android.widget.ImageView;
import android.widget.ImageView.ScaleType;

public class RosbeeRemoteActivity extends Activity  implements AccelerometerListener {
	
	private static UDPClient _client;
	private static Context CONTEXT;
	private RecvThread _RecvThread;
	private Img_RecvThread rt;
	private ImageView imgView;
	private boolean running;
	
	private final int SteeringPort = 1234;
	private final int ImagePort= 1337;
	private final String IP = "192.168.1.151";
	
	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
       
        running = false;
        CONTEXT = this;       
        setContentView(R.layout.main);
      
        _client = UDPClient.GetInstance("10.10.0.10",SteeringPort); 
        imgView = (ImageView)findViewById(R.id.imgView);
		imgView.setScaleType(ScaleType.FIT_XY);
        
		
		//Error Thread
        _RecvThread = new RecvThread(_client.getServerSock(),getContext());
		_RecvThread.start();
		
		
		//Image Thread
		/*rt = new Img_RecvThread(IP, ImagePort, new Handler(){
			
			@Override
			public void dispatchMessage(Message msg) {
				RosbeeRemoteActivity.this.UpdateImage(rt.GetImage());
				super.dispatchMessage(msg);
			}
			
		});
		rt.start(); */
	}
    
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
	    MenuInflater inflater = getMenuInflater();
	    inflater.inflate(R.layout.optionmenu, menu);
	    return true;
	}
	
	@Override public boolean onPrepareOptionsMenu(Menu menu) 
	{
		if(running)
		menu.getItem(1).setTitle("Stop");
		else
		menu.getItem(1).setTitle("Start");	
			return true;
	}
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) 
		{
		case R.id.StartStop_ITEM:
			if(running)
				Stop();
			else
				Start();
			break;
		case R.id.EXIT_ITEM:
			Stop();
			this.finish();			
			break;
		}
		return true;
		}
	
	public void Start()
	{
		if(running == true)
			return;
		
		running =true;
		AccelerometerManager.startListening(this);
	}
	public void Stop()
	{
		if(running == false)
			return;
		
		running = false;
		AccelerometerManager.stopListening();
	}
    public static Context getContext() {
                return CONTEXT;
        }
   	
	@Override 
	    protected void onDestroy() {
	    	super.onDestroy();
	    	Stop();  	
	    }
	    
	public void onAccelerationChanged(float x, float y, float z) {
	
		_client.sendUDPString(x+";"+y+";"+z);
	}
	
	private void UpdateImage(Bitmap bmp)
	{
		imgView.setImageBitmap(bmp);
	}
	

}