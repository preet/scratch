package example.app.android.displaysettings;

import android.app.Activity;
import android.content.res.Configuration;
import android.os.Build;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Display;
import android.view.Surface;
import android.view.ViewTreeObserver;
import android.widget.RelativeLayout;
import android.widget.TextView;


public class MainActivity extends Activity {

    private static final String TAG = "MainActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Get Display Info
        getDisplayInfo();

        // Get Window Size
        m_window_width_px = 0;
        m_window_height_px = 0;
        addLayoutSizeChangeListener();
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig)
    {
        super.onConfigurationChanged(newConfig);

        Log.v(TAG, "Configuration changed!");

        final RelativeLayout rootLayout = (RelativeLayout) findViewById(R.id.root_layout);

        // Get Display Rotation
        getDisplayRotation();

        // Get the Window size
        addLayoutSizeChangeListener();
    }

    private void addLayoutSizeChangeListener()
    {
        // ref:
        // http://stackoverflow.com/a/30469196
        final RelativeLayout rootLayout = (RelativeLayout) findViewById(R.id.root_layout);

        rootLayout.getViewTreeObserver().addOnGlobalLayoutListener(
                new ViewTreeObserver.OnGlobalLayoutListener() {
                    @Override
                    public void onGlobalLayout() {

                        int new_window_width_px = rootLayout.getWidth();
                        int new_window_height_px = rootLayout.getHeight();

                        if (new_window_width_px != m_window_width_px ||
                                new_window_height_px != m_window_height_px) {
                            if (Build.VERSION.SDK_INT >= 16) {
                                rootLayout.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                            } else {
                                rootLayout.getViewTreeObserver().removeGlobalOnLayoutListener(this);
                            }

                            m_window_width_px = rootLayout.getWidth();
                            m_window_height_px = rootLayout.getHeight();

                            Log.v(TAG, "Window dims:");
                            Log.v(TAG, "Window width: " + m_window_width_px);
                            Log.v(TAG, "Window height: " + m_window_height_px);

                            resetInfoText();
                        }
                    }
                }
        );
    }

    private void getDisplayInfo()
    {
        Display display = getWindowManager().getDefaultDisplay();

        if(Build.VERSION.SDK_INT >= 17) {
            m_display_name = display.getName();

            DisplayMetrics metrics = new DisplayMetrics();
            display.getRealMetrics(metrics);

            m_display_width_px = metrics.widthPixels;
            m_display_height_px = metrics.heightPixels;

            m_display_xdpi = metrics.xdpi;
            m_display_ydpi = metrics.ydpi;
        }
        else {
            m_display_name = Integer.toString(display.getDisplayId());

            DisplayMetrics metrics = new DisplayMetrics();
            display.getMetrics(metrics);

            // ref: http://stackoverflow.com/questions/2193457/...
            // ...is-there-a-way-to-determine-android-physical-screen-height-in-cm-or-inches
            // since SDK_INT = 1;
            m_display_width_px = metrics.widthPixels;
            m_display_height_px = metrics.heightPixels;

            // includes window decorations (statusbar bar/menu bar)
            if (Build.VERSION.SDK_INT >= 14)  {
                try  {
                    m_display_width_px = (Integer) Display.class.getMethod("getRawWidth").invoke(display);
                    m_display_height_px = (Integer) Display.class.getMethod("getRawHeight").invoke(display);
                }
                catch (Exception ignored)
                { }
            }

            m_display_xdpi = metrics.xdpi;
            m_display_ydpi = metrics.ydpi;
        }

        getDisplayRotation();
    }

    private void getDisplayRotation()
    {
        Display display = getWindowManager().getDefaultDisplay();
        DisplayMetrics metrics = new DisplayMetrics();
        display.getMetrics(metrics);

        final int rotation = rotationEnumToDegs(display.getRotation());
        int rotation_cw;
        if(rotation == 0) {
            rotation_cw = 0;
        }
        else {
            rotation_cw = 360-rotation;
        }

        if(rotation_cw != m_display_rotation_cw) {
            m_display_rotation_cw = rotation_cw;
            resetInfoText();
        }
    }

    private int rotationEnumToDegs(int rotation_enum)
    {
        if(rotation_enum == Surface.ROTATION_0) {
            return 0;
        }
        else if(rotation_enum == Surface.ROTATION_90) {
            return 90;
        }
        else if(rotation_enum == Surface.ROTATION_180) {
            return 180;
        }

        return 270;
    }

    private void resetInfoText()
    {
        String info="";
        info = info.concat("Display Info:\n");
        info = info.concat("name: " + m_display_name +"\n");
        info = info.concat("width_px: " + m_display_width_px +"\n");
        info = info.concat("height_px: " + m_display_height_px +"\n");
        info = info.concat("xdpi: " + m_display_xdpi +"\n");
        info = info.concat("ydpi: " + m_display_ydpi +"\n");
        info = info.concat("rotation_cw: " + m_display_rotation_cw +"\n");

        info = info.concat("\nWindow Info:\n");
        info = info.concat("width_px: " + m_window_width_px+"\n");
        info = info.concat("height_px: " + m_window_height_px+"\n");

        TextView textView = (TextView) findViewById(R.id.info_text);
        textView.setText(info);
    }

    // Display Info
    private String m_display_name;
    private int m_display_width_px;
    private int m_display_height_px;
    private float m_display_xdpi;
    private float m_display_ydpi;
    private int m_display_rotation_cw; // 0,90,180,270

    // Window (root view) Info
    private int m_window_width_px;
    private int m_window_height_px;
}
