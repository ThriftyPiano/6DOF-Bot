package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Environment;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class ImageSaver {

    private static final String TAG = "ImageSaver";

    public static boolean saveMatToDisk(Mat mat, String filename) {
        Bitmap bitmap = null;
        FileOutputStream fos = null;

        try {
            // 1. Convert Mat to Bitmap
            bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, bitmap);

            // 2. Access the SD Card
            File sdCard = Environment.getExternalStorageDirectory();
            if (sdCard == null) {
                Log.e(TAG, "SD card not found.");
                return false;
            }

            // 3. Create a File Object
            File directory = new File(sdCard.getAbsolutePath() + "/FIRST/Images"); // Create a directory if needed
            if (!directory.exists()) {
                if (!directory.mkdirs()) {
                    Log.e(TAG, "Failed to create directory: " + directory.getAbsolutePath());
                    return false;
                }
            }
            File imageFile = new File(directory, filename + ".jpg"); // You can change the extension

            // 4. Create an Output Stream
            fos = new FileOutputStream(imageFile);

            // 5. Compress and Write the Bitmap
            // You can choose different formats (JPEG, PNG) and quality
            bitmap.compress(Bitmap.CompressFormat.JPEG, 90, fos); // JPEG with 90% quality

            Log.i(TAG, "Image saved to: " + imageFile.getAbsolutePath());
            return true;

        } catch (IOException e) {
            Log.e(TAG, "Error saving image: " + e.getMessage());
            return false;
        } finally {
            // 6. Close the Output Stream
            if (fos != null) {
                try {
                    fos.close();
                } catch (IOException e) {
                    Log.e(TAG, "Error closing output stream: " + e.getMessage());
                }
            }
            if (bitmap != null) {
                bitmap.recycle(); // Release bitmap resources
            }
        }
    }
}
