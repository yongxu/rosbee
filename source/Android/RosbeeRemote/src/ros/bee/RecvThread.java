package ros.bee;

import java.net.DatagramSocket;

public class RecvThread implements Runnable {

	private DatagramSocket _s;
	
	public RecvThread(DatagramSocket sock)
	{
		_s = sock;		
	}
	
	public void run() {
		UDPClient.ReceiveUDP(_s);
	}

}
