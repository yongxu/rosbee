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
	
	public RecvThread(DatagramSocket sock, Handler handler)
	{
		_s = sock;	
		_recv = "";
		_handler = handler;
		run = true;
		
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
	}
	
	boolean StopThread() {
		run = false;
		while(!run) //sleep until the thread ended
		{
			try {
				sleep(10);
			} catch (InterruptedException e) {
				return false;
			}
		}
		return true;
	}

}
