package ros.bee;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;

import android.R.color;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Handler;
import android.widget.ImageView;

public class Img_RecvThread implements Runnable {
	private DatagramSocket _socket;
	private ImageView _imgview;
	private int _port;
	private InetAddress _ip;
	private volatile Bitmap bmp;
	
	private Handler _imgHandler;
	
	private final int IMGHEIGHT = 120;
	private final int IMGWIDTH = 160;
	private final int BUFFSIZE = IMGWIDTH * IMGHEIGHT * 3;
	
	public Img_RecvThread(String Ip, int port, ImageView view, Handler imghandler) {
		try {
			_socket = new DatagramSocket(port);
			_ip = InetAddress.getByName(Ip);
			_port = port;
			_imgHandler = imghandler;
		}
		catch (SocketException e) {
			System.out.println("socket exception!!!" + e.toString());
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		bmp = Bitmap.createBitmap(IMGWIDTH,IMGHEIGHT,Bitmap.Config.ARGB_8888); 
		_imgview = view;
		
	}

	public void run() {
		byte[] buffer = new byte[BUFFSIZE];
		while(true)
		{
			try {
				DatagramPacket packet = new DatagramPacket(buffer, BUFFSIZE);
				_socket.receive(packet);
			} 
			catch (IOException e) {
				System.out.println(e.toString());
			}
			
			int row = 0, col = 0;
			for(int i = 0; i < BUFFSIZE; i+=3)
			{
				bmp.setPixel(col, row,Color.rgb(buffer[i],buffer[i+1],buffer[i+2]));
				col++;
				if(col == IMGWIDTH)
				{
					row++;
					col = 0;
				}
			}
			System.out.println("img");
			_imgHandler.sendEmptyMessage(0);
		}
	}
	
	Bitmap GetImage()
	{
		return bmp;
	}
	

}
