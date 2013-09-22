package ca.predesign.prismatic;

import org.qtproject.qt5.android.bindings.QtActivity;
import android.os.Bundle;
import android.util.Log;

public class PrisActivity extends QtActivity
{
//---------------------------------------------------------------------------
    public native void activityReady();

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        Log.i("PrisActivity: ","####: onCreate After\n\n\n");

        activityReady();
    }
//---------------------------------------------------------------------------
}
