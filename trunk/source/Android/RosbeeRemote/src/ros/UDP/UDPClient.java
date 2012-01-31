package ros.UDP;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;

import ros.bee.RecvThread;

import android.widget.Toast;

public class UDPClient {

	private static UDPClient _client;
	private DatagramSocket _sock;
	private InetAddress IP;
	private static final int BUFFERSIZE = 40;
	private int PORT;
	


	private UDPClient(String Ip, int port)
	{ 
		try 
		{
			System.out.println("udp geneuzel");
			_sock = new DatagramSocket();
			IP = InetAddress.getByName(Ip);
			PORT = port;
			
			  
		} 
		catch (Exception e) 
		{
			System.out.println("kennie connecten nie!");
			System.out.println(e.toString());
			//toast.show();
		}
	}
	public DatagramSocket getSeverSock()
	{
		return _sock;
		
	}
	public void sendUDPString(String s)
	{
		if(_sock == null)
			return;
		
		byte[] buffer = new byte[BUFFERSIZE];
		
		//string naar buffer kopieren. + zorgen dat de buffer evengroot als buffersize blijft.
		int length;		
		if(s.length() > BUFFERSIZE)
		{
			length = BUFFERSIZE;
		}
		else
		{
			length = s.length();
		}
		for(int i = 0; i < length; i++)
		{
			buffer[i] = s.getBytes()[i];
		}
		
		try
		{			
			DatagramPacket sendPacket = new DatagramPacket(buffer, BUFFERSIZE, IP, PORT);
		    _sock.send(sendPacket);
			//System.out.println(buffer.toString());
		}
		catch(Exception ex)
		{
			System.out.println("kennie senden nie!");
			System.out.println(ex.toString());
		}
	}
	
	public static String ReceiveUDP(DatagramSocket s)
	{
		if(s == null)
			return "" ;
		
		byte[] buffer = new byte[BUFFERSIZE];
		
		try
		{		
		 DatagramPacket receivePacket = new DatagramPacket(buffer,BUFFERSIZE);
	      s.receive(receivePacket);	        
		}
		catch(Exception ex)
		{
			System.out.println("kennie recieven nie!");
		}
		String str = new String(buffer);
		str.trim();		
		 return str;
	}


	public static UDPClient GetInstance(String ip, int Port)
	{
		if(_client == null)
			_client = new  UDPClient(ip,Port);

		return _client;
	}
	
	public void Close()
	{
		_sock.close();
	}

	
	
}
