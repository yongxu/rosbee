package ros.bee;

import java.net.DatagramSocket;

import android.content.Context;
import android.os.Handler;
import android.widget.TextView;
import android.widget.Toast;

import ros.UDP.UDPClient;

public class RecvThread extends Thread  {

	private DatagramSocket _s;
	private String _recv;
	private Handler _handler;
	
	public RecvThread(DatagramSocket sock, Handler handler)
	{
		_s = sock;	
		_recv = "";
		_handler = handler;
	}
	
	public String getReceivedMessage()
	{
		return _recv;
	}
	
	public void run() {
		while(true)
		{
					
			try
			{
				_recv = (UDPClient.ReceiveUDP(_s));
			}
			catch (Exception e) {
			System.out.println(e.toString());
			
			}
	
			_handler.sendEmptyMessage(0);
		}
	}

}
