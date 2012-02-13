package ros.UDP;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;


public class UDPClient {

	private static UDPClient _client;
	private DatagramSocket _sock;
	private InetAddress IP;
	private static final int BUFFERSIZE = 70;
	private int PORT;
	

	private UDPClient(String Ip, int port)
	{ 
		try 
		{
		
			IP = InetAddress.getByName(Ip);
			PORT = port;
			_sock = new DatagramSocket(port,IP);
			  
		} 
		catch (Exception e) 
		{
		
			System.out.println(e.toString());
			//toast.show();
		}
	}
	public DatagramSocket getServerSock()
	{
		return _sock;
		
	}
	
	public void setPort(int port)
	{
		PORT = port;
	}
	public void SetIp(String ip)
	{
		try {
			IP = InetAddress.getByName(ip);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
			DatagramPacket sendPacket = new DatagramPacket(buffer, BUFFERSIZE,IP,PORT);
		    _sock.send(sendPacket);
		}
		catch(Exception ex)
		{
			//System.out.print("keiharde poep");
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
			System.out.println(ex.toString());
		}
		String str = new String(buffer);
		str =str.trim();		
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
