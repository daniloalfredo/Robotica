opencv_libs = -I/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_video -lopencv_highgui -lopencv_photo -lopencv_flann -lopencv_stitching -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_core -lopencv_legacy

file = colorTracking.cpp

main: $(file)
	g++ $(file) -o main $(opencv_libs) 

clean:
	rm main
