package ros.bee;

import java.io.IOException;
import java.io.InterruptedIOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Handler;

public class Img_RecvThread extends Thread {
	private DatagramSocket _socket;
	private int _port;
	private volatile Bitmap bmp;
	private volatile boolean run;
  private volatile boolean stopped;

	private Handler _imgHandler;

	private final int IMGHEIGHT = 120;
	private final int IMGWIDTH = 160;
	private final int BUFFSIZE = IMGWIDTH * IMGHEIGHT * 3;
	private final int SOCKTIMEOUT = 100; // timout for the socket in ms

	public Img_RecvThread(int port, Handler imghandler) {
		try {
			_socket = new DatagramSocket(port);
			_port = port;
			_imgHandler = imghandler;
			_socket.setSoTimeout(SOCKTIMEOUT);
			run = true;
      stopped = false;

			System.out.println("port: " + _port);
		} catch (SocketException e) {
			System.out.println("socket exception!!!" + e.toString());
		}
		bmp = Bitmap.createBitmap(IMGWIDTH, IMGHEIGHT, Bitmap.Config.ARGB_8888);

	}

	public void run() {
		byte[] buffer = new byte[BUFFSIZE];
		boolean readed = true;
    stopped = false;
    run = true;
    
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
		stopped = true;
	}

	Bitmap GetImage() {
		return bmp;
	}

	boolean StopThread() {
		run = false;
		while(!stopped) //sleep until the thread ended
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

