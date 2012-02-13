package ros.bee;


import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;


public class ConfigActivity extends  Activity{

	 @Override
	    public void onCreate(Bundle savedInstanceState) {
	        super.onCreate(savedInstanceState);	        
	        setContentView(ros.bee.R.layout.configmenu);
	      
	     ((EditText) findViewById(R.id.et_control_ip)).setText(getIntent().getStringExtra("IP")); 
	    ((EditText) findViewById(R.id.et_img_port)).setText(getIntent().getStringExtra("IMAGEPORT")); 
	    ((EditText) findViewById(R.id.et_control_port)).setText(getIntent().getStringExtra("CONTROLPORT")); 
	 }
	 public void submitClick(View v)
		{
		 String ip = ((EditText) findViewById(R.id.et_control_ip)).getText().toString();
		 String imageport = ((EditText) findViewById(R.id.et_img_port)).getText().toString();
		 String controlport = ((EditText) findViewById(R.id.et_control_port)).getText().toString();
		 
		 Intent intent = new Intent();
		 intent.putExtra("IP", ip );
		 intent.putExtra("IMAGEPORT", imageport );
		 intent.putExtra("CONTROLPORT", controlport );
		 
         setResult(RESULT_OK, intent);
         finish();
		}
		
		public void CancelClick(View v)
		{
			Intent intent = new Intent();
	         setResult(RESULT_CANCELED, intent);
	         finish();
		}
		
}
