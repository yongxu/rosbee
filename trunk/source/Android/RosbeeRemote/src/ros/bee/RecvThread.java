package ros.bee;

import java.net.DatagramSocket;

import android.widget.TextView;

import ros.UDP.UDPClient;

public class RecvThread extends Thread  {

	private DatagramSocket _s;
	private TextView  _tv;
	
	public RecvThread(DatagramSocket sock, TextView tv)
	{
		_s = sock;		
		_tv = tv;
	}
	
	public void run() {
		while(true)
		{
			try
			{
					_tv.setText(UDPClient.ReceiveUDP(_s));
			}
			catch (Exception e) {
			System.out.print(e.toString());
			}
	
		
		}
	}

}
