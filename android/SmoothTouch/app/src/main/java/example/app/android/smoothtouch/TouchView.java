package example.app.android.smoothtouch;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

public class TouchView extends View {

    private static final String TAG = "TouchView";

    private Paint mPaintBackground = new Paint();
    private Paint mPaintTouch = new Paint();
    private float mTouchSize = 100;
    private float mTouchPointX = 0;
    private float mTouchPointY = 0;

    public TouchView(Context context, AttributeSet attrs) {
        super(context,attrs);
        mPaintBackground.setColor(Color.rgb(25, 25, 25));
        mPaintTouch.setColor(Color.rgb(180,25,25));
    }

    @Override
    public void onDraw(Canvas canvas) {
        canvas.drawRect(0, 0, getWidth(), getHeight(), mPaintBackground);

        canvas.drawRect(
                mTouchPointX - (mTouchSize*0.5f),
                mTouchPointY - (mTouchSize*0.5f),
                mTouchPointX + mTouchSize,
                mTouchPointY + mTouchSize,
                mPaintTouch);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mTouchPointX = event.getX();
        mTouchPointY = event.getY();
        invalidate();

        return true;
    }

}
