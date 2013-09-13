package org.qtproject.qt5.android.bindings;

import android.os.Bundle;
import android.util.Log;

public class MyQtActivity extends QtActivity
{
//---------------------------------------------------------------------------
    public native void activityReady();

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        Log.i("My_Activity: ","####: onCreate After");

        activityReady();
    }
//---------------------------------------------------------------------------
}
