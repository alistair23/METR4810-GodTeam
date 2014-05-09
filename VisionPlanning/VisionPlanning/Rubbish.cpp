
	/*
	// Find contours for each canny
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	// Only keep contours which approximate quadrilaterals and are above a certain size
	std::vector<std::vector<cv::Point>> corners;		// Corner points
	std::vector<cv::Point> centres;			// Centre points
	for (std::vector<cv::Point> contour : contours)
	{
		cv::Mat contour_mat = cv::Mat(contour);
		std::vector<cv::Point> result;
		cv::approxPolyDP(contour_mat, result, 5, true);	// Approximate the contour as a polygon

		// Check we have 4 points (quadrilateral) and is above area requirement
		if (result.size() == 4) // && contourArea(result) > 200)
		{
			cv::Point centre = findCentre(result);
			centres.push_back(centre);
			corners.push_back(result);
		}
	}

	cv::Mat cons;
	cv::cvtColor(dst, cons, CV_GRAY2BGR);

	for (std::size_t i = 0; i < corners.size(); i++) {
		for (std::size_t j = 0; j < corners[i].size(); j++) {
			cv::line(cons, corners[i][j], corners[i][(j + 1) % corners[i].size()], cv::Scalar(255,0,0), 3, 8);
		}
	}

	cv::imshow("Display", cons);
	cv::waitKey();
	*/
/*
	// Find lines from Canny image
	std::vector<cv::Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 150, 30, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::line( cdst, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
    }

	cv::imshow("Display", cdst);
	cv::waitKey();
	*/
/*
#include<opencv2/opencv.hpp>
 
using namespace cv;
 
int main( )
{
    // Input Quadilateral or Image plane coordinates
    Point2f inputQuad[4]; 
    // Output Quadilateral or World plane coordinates
    Point2f outputQuad[4];
         
    // Lambda Matrix
    Mat lambda( 2, 4, CV_32FC1 );
    //Input and Output Image;
    Mat input, output;
     
    //Load the image
    input = imread( "lena.jpg", 1 );
    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros( input.rows, input.cols, input.type() );
 
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    inputQuad[0] = Point2f( -30,-60 );
    inputQuad[1] = Point2f( input.cols+50,-50);
    inputQuad[2] = Point2f( input.cols+100,input.rows+50);
    inputQuad[3] = Point2f( -50,input.rows+50  );  
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = Point2f( 0,0 );
    outputQuad[1] = Point2f( input.cols-1,0);
    outputQuad[2] = Point2f( input.cols-1,input.rows-1);
    outputQuad[3] = Point2f( 0,input.rows-1  );
 
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    warpPerspective(input,output,lambda,output.size() );
 
    //Display input and output
    imshow("Input",input);
    imshow("Output",output);
 
    waitKey(0);
    return 0;
}
*/

/*
std::vector<Point> Vision::getMidpoints(cv::Mat& img_thresh,
										cv::Point2f start_pos,
										float start_dir) {
											
	cv::Mat img_temp, grad_x, grad_y;
	cv::blur(img_thresh, img_temp, cv::Size(10, 10));

	cv::imshow("Display", img_temp);
	cv::waitKey();

	// Use Sobel operator to get image derivatives
	cv::Sobel(img_temp, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
	cv::Sobel(img_temp, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

	// Make color format for showing stuff
	//cv::Mat cdst;
	//cv::cvtColor(img_thresh, cdst, CV_GRAY2BGR);

	// Find first midpoint


	std::vector<Point> midpoints;
	midpoints.push_back(Point(start_pos.x, start_pos.y));
	float expected_width = ROAD_WIDTH / M_PER_PIX;
	
	float width_thresh = 0.1;	// 10 % for accepting result
	float step_size_sq = GLOBAL_PATH_STEP_SIZE * GLOBAL_PATH_STEP_SIZE;

	// Loop until image edge is reached or until we loop back into starting point
	while (true) {
			// Extrapolate forward using last point
	float temp_x = midpoints.back().x + GLOBAL_PATH_STEP_SIZE * cos(angles.back());
	float temp_y = midpoints.back().y + GLOBAL_PATH_STEP_SIZE * sin(angles.back());
	cv::Point2f temp_p(temp_x, temp_y);
		
	}

	//cv::imshow("Display", cdst);
	//cv::waitKey();
	

	return midpoints;
	
	return std::vector<Point>();
}

Point Vision::getMidpoint(cv::Mat& img_thresh, cv::Mat& img_sobel_x, 
						  cv::Mat& img_sobel_y, cv::Point2f start_pos,
						  float start_dir, bool& success) {

	// Find the road edge on the right
	bool at_img_edge;
	cv::Point2f b1 = getBoundary(img_thresh, start_pos, start_dir + M_PI_2, at_img_edge);
	if (at_img_edge) {
		success = false;
		return Point(0, 0);
	}
	
	// Determine the gradient at the boundary point found
	// (e.g. for a straight road east to west, dx = 0, dy > 0)
	short dx = -img_sobel_x.at<short>((int)b1.y, (int)b1.x);
	short dy = -img_sobel_y.at<short>((int)b1.y, (int)b1.x);
	float gradient = atan2(dy, dx);

	// Find left road edge by following the gradient from b1
	cv::Point2f b2 = getBoundary(img_thresh, b1, gradient, at_img_edge);
	if (at_img_edge) {
		success = false;
		return Point(0, 0);
	}

	float width = dist(b1, b2);

	cv::Point2f new_p_cv = b1 + ((b2 - b1) * 0.5);
	Point new_p(new_p_cv.x, new_p_cv.y, gradient + M_PI_2, dist(new_p_cv, b1), dist(new_p_cv, b2));
	//angles.push_back(gradient + M_PI_2);
	//midpoints.push_back(new_p);
	//widths.push_back(width);
}
*/