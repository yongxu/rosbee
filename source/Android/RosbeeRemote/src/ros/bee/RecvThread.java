package ros.bee;

import java.net.DatagramSocket;

import android.content.Context;
import android.os.Handler;
import android.widget.TextView;
import android.widget.Toast;

import ros.UDP.UDPClient;

public class RecvThread extends Thread  {

	private DatagramSocket _s;
	private Context _context;
	
	public RecvThread(DatagramSocket sock, Context tex)
	{
		_s = sock;	
		_context = tex;

	}
	
	public void run() {
		while(true)
		{
					
			try
			{
				//Toast.makeText(_context,UDPClient.ReceiveUDP(_s) , Toast.LENGTH_SHORT).show();	
				//System.out.println(UDPClient.ReceiveUDP(_s));
			}
			catch (Exception e) {
			System.out.println(e.toString());
			
			}
	
	
		}
	}

}
