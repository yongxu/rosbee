package ros.Acellerometer;

import java.security.Timestamp;
import java.sql.Date;
import java.util.List;

import ros.bee.RosbeeRemoteActivity;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;


public class AccelerometerManager {
	

	private static int pauze_ns = 100000000;

	private static Sensor sensor;
	private static SensorManager sensorManager;
	private static AccelerometerListener listener;
	
	private static Boolean supported;
	private static boolean running = false;
	
	/**
	 * Returns true if the manager is listening to orientation changes
	 */
	public static boolean isListening() {
		return running;
	}
	
	/**
	 * Unregisters listeners
	 */
	public static void stopListening() {
		running = false;
		try {
			if (sensorManager != null && sensorEventListener != null) {
				sensorManager.unregisterListener(sensorEventListener);
			}
		} catch (Exception e) {}
	}
	
	/**
	 * Returns true if at least one Accelerometer sensor is available
	 */
	public static boolean isSupported() {
		if (supported == null) {
			if (RosbeeRemoteActivity.getContext() != null) {
				sensorManager = (SensorManager) RosbeeRemoteActivity.getContext().
						getSystemService(Context.SENSOR_SERVICE);
				List<Sensor> sensors = sensorManager.getSensorList(
						Sensor.TYPE_ACCELEROMETER);
				supported = new Boolean(sensors.size() > 0);
			} else {
				supported = Boolean.FALSE;
			}
		}
		return supported;
	}


	public static void startListening(AccelerometerListener accelerometerListener) {
		sensorManager = (SensorManager) RosbeeRemoteActivity.getContext().
				getSystemService(Context.SENSOR_SERVICE);
		List<Sensor> sensors = sensorManager.getSensorList(
				Sensor.TYPE_ACCELEROMETER);
		if (sensors.size() > 0) {
			sensor = sensors.get(0);
			running = sensorManager.registerListener(
					sensorEventListener, sensor, 
					SensorManager.SENSOR_DELAY_GAME);
			listener = accelerometerListener;
		}
	}

	/**
	 * The listener that listen to events from the accelerometer listener
	 */
	private static SensorEventListener sensorEventListener = 
		new SensorEventListener() {

		private long now = 0;
		private long lastUpdate = 0;
	
		private float x = 0;
		private float y = 0;
		private float z = 0;
		
		public void onAccuracyChanged(Sensor sensor, int accuracy) {}
		
		public void onSensorChanged(SensorEvent event) {
			
			now = event.timestamp;
					
			if(now - lastUpdate > pauze_ns)
			{
	    		x = event.values[0];
				y = event.values[1];
				z = event.values[2];
	    		
	    		// trigger change event
	    		listener.onAccelerationChanged(x, y, z);
	    		lastUpdate = now;
			}
		}
		
	};

}