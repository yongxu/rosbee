package ros.bee;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Handler;

public class Img_RecvThread extends Thread {
	private DatagramSocket _socket;
	private int _port;
	private InetAddress _ip;
	private volatile Bitmap bmp;
	private volatile boolean run;

	private Handler _imgHandler;

	private final int IMGHEIGHT = 120;
	private final int IMGWIDTH = 160;
	private final int BUFFSIZE = IMGWIDTH * IMGHEIGHT * 3;
	private final int SOCKTIMEOUT = 100; // timout for the socket in ms

	public Img_RecvThread(String Ip, int port, Handler imghandler) {
		try {
			_socket = new DatagramSocket(port);
			_ip = InetAddress.getByName(Ip);
			_port = port;
			_imgHandler = imghandler;
			_socket.setSoTimeout(SOCKTIMEOUT);
			run = true;

			System.out.println("IP: " + _ip.toString() + " port: " + _port);
		} catch (SocketException e) {
			System.out.println("socket exception!!!" + e.toString());
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		bmp = Bitmap.createBitmap(IMGWIDTH, IMGHEIGHT, Bitmap.Config.ARGB_8888);

	}

	public void run() {
		byte[] buffer = new byte[BUFFSIZE];
		boolean readed = true;
		while (run) {
			readed = true;
			try {
				DatagramPacket packet = new DatagramPacket(buffer, BUFFSIZE);
				_socket.receive(packet);

			} catch (InterruptedIOException e) {
				if (e.bytesTransferred < IMGHEIGHT * IMGWIDTH * 3)
					readed = false;
			} catch (IOException e) {
				System.out.println(e.toString());
				readed = false;
			}

			if (readed) {
				int row = 0, col = 0;
				for (int i = 0; i < BUFFSIZE; i += 3) {
					bmp.setPixel(col, row,
							Color.rgb(buffer[i], buffer[i + 1], buffer[i + 2]));
					col++;
					if (col == IMGWIDTH) {
						row++;
						col = 0;
					}
				}
				System.out.println("img");
				_imgHandler.sendEmptyMessage(0);
			}
		}
		run = true;
	}

	Bitmap GetImage() {
		return bmp;
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

