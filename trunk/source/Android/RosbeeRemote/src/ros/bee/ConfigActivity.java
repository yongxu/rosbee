package ros.bee;

import android.R;
import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

public class ConfigActivity extends  Activity{
	Button submit;
	Button cancel;
	 @Override
	    public void onCreate(Bundle savedInstanceState) {
	        super.onCreate(savedInstanceState);
	        
	        setContentView(ros.bee.R.layout.configmenu);
	        
	         submit = (Button) findViewById(ros.bee.R.id.btn_submit);
	         cancel  = (Button) findViewById(ros.bee.R.id.btn_cancel);
	        
	        
	         submit.setOnClickListener(new View.OnClickListener() {
	            public void onClick(View view) {
	                Intent intent = new Intent();
	                setResult(RESULT_OK, intent);
	                finish();
	            }

	        });
	         
	         cancel.setOnClickListener(new View.OnClickListener() {
		            public void onClick(View view) {
		                Intent intent = new Intent();
		                setResult(RESULT_OK, intent);
		                finish();
		            }

		        });
	        
	      
	 }
}
