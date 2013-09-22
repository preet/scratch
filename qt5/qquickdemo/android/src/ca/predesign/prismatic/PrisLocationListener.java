package ca.predesign.prismatic;

import android.os.Bundle;
import android.util.Log;
import android.location.Location;
import android.location.LocationManager;
import android.location.LocationListener;

//-----------------------------------------------------------------------//

public class PrisLocationListener implements LocationListener
{
//public native void activityReady();

   // where 'status' determines the validity of
   // the geolocation information according to
   // the following values:
   
   // ALL INVALID:               0
   // TIME VALID:                1
   // LONLAT VALID:              2
   // ALT VALID:                 4
   // SPEED VALID:               8
   // HEADING VALID:             16
   // LONLAT ERROR VALID:        32
   // ALL VALID:                 64
   
   public native void nativeOnLocationChanged(long utc_time,
                                         double lon,
                                         double lat,
                                         double alt,
                                         double speed,
                                         double heading,
                                         double err_lonlat,
                                         int status);
                                         
   public native void nativeOnStatusChanged(int status);
   
   public native void nativeOnProviderEnabled(int provider);
   
   public native void nativeOnProviderDisabled(int provider);

   public void onLocationChanged(Location location)
   {
      //Log.i("PrisLocationListener: ","####: onLocationChanged Called!");
      this.nativeOnLocationChanged(location.getTime(),
                                   location.getLongitude(),
                                   location.getLatitude(),
                                   location.getAltitude(),
                                   location.getSpeed(),
                                   location.getBearing(),
                                   location.getAccuracy(),
                                   0);
   }

   public void onStatusChanged(String provider, int status, Bundle extras) 
   {
      //Log.i("PrisLocationListener: ","####: onStatusChanged Called!");
      this.nativeOnStatusChanged(status);
   }

   public void onProviderEnabled(String provider)
   {
      //Log.i("PrisLocationListener: ","####: onProviderEnabled Called!");
      int i_provider=99;
      if(provider == LocationManager.GPS_PROVIDER)   {
         i_provider=0;
      }
      else if(provider == LocationManager.NETWORK_PROVIDER)   {
         i_provider=1;
      }
      else if(provider == LocationManager.PASSIVE_PROVIDER)   {
         i_provider=2;
      }
      
      this.nativeOnProviderEnabled(i_provider);
   }
   
   public void onProviderDisabled(String provider)
   {
      //Log.i("PrisLocationListener: ","####: onProviderDisabled Called!");
      
      int i_provider=99;
      if(provider == LocationManager.GPS_PROVIDER)   {
         i_provider=0;
      }
      else if(provider == LocationManager.NETWORK_PROVIDER)   {
         i_provider=1;
      }
      else if(provider == LocationManager.PASSIVE_PROVIDER)   {
         i_provider=2;
      }
      
      this.nativeOnProviderDisabled(i_provider);
   }
}

//-----------------------------------------------------------------------//
