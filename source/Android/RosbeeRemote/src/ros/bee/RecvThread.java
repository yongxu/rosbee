package ros.bee;

import java.net.DatagramSocket;


import android.os.Handler;


import ros.UDP.UDPClient;

public class RecvThread extends Thread  {

	private DatagramSocket _s;
	private String _recv;
	private Handler _handler;
	private volatile boolean run;
	private final int SOCKTIMEOUT = 100;
	 private volatile boolean stopped;
	 
	public RecvThread(DatagramSocket sock, Handler handler)
	{
		_s = sock;	
		_recv = "";
		_handler = handler;
		run = true;
		  stopped = false;
		try
		{
		_s.setSoTimeout(SOCKTIMEOUT);
		}
		catch(Exception ex)
		{
			
		}
	}
	
	public String getReceivedMessage()
	{
		return _recv;
	}
	
	public void run() {
		while(run)
		{
					
			try
			{
				_recv = (UDPClient.ReceiveUDP(_s));
			}
			catch (Exception e) {
			//System.out.println(e.toString());
			
			}
	
			_handler.sendEmptyMessage(0);
		}
		stopped = true;
		_s.close();
	}
	
	boolean StopThread() {
		run = false;
		while(!stopped) //sleep until the thread ended
		{
			try {
				sleep(50);
			} catch (InterruptedException e) {
				return false;
			}
		}
		return true;
	}

}
