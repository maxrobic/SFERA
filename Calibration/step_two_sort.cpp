/****************************************************************************************************
* This code allows the mathing of pixel coordinates with 3D world coordinates.                      *
* Initially, the algorithm proposes an automatic association.                                       *
* The user must then validate the coordinate matching for each pose.                                *
* In case of incorrect matching, the user has the option to adjust the data                         *
* (in case of singularity issues) by clicking on the four corners of the calibration grid.          *                                               *
* Please follow the instructions regarding the order of the points to be clicked.                   *
* This code takes as input :                                                                        *
* - The image data                                                                                  *
* - Pixel position data                                                                             *
* - The distance between 2 points of interest                                                       *
* - The number of rows                                                                              *
* - The number of columns                                                                           *
*                                                                                                   *
* This code is part of the calibration process used in the context of "A New Stereo Fisheye Event   *
* Camera for Fast Drone Detection and Tracking - Daniel Rodrigues Da Costa, Maxime Robic "          ∗  
****************************************************************************************************/
  
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

void drawPoints(cv::Mat& image, const std::vector<cv::Point2d>& points, const cv::Scalar& color) {
    for (size_t i = 0; i < points.size(); ++i) {
        cv::circle(image, points[i], 5, color, -1); // Draw a circle with the specified colour
        cv::putText(image, std::to_string(i + 1), points[i], cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 2);
    }
}

void drawArrows(cv::Mat& image, const std::vector<cv::Point2d>& points) {
 
    if (points.size() >= 4) {
        cv::arrowedLine(image, points[0], points[1], cv::Scalar(0, 0, 255), 2);
        cv::putText(image, "X", points[1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::arrowedLine(image, points[0], points[3], cv::Scalar(0, 0, 255), 2);
        cv::putText(image, "Y", points[3], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }
}


int findNearestInputPoint(const std::vector<cv::Point2d>& inputPoints, const cv::Point2d& clickedPoint) {
    double minDist = std::numeric_limits<double>::max();
    int nearestIndex = -1;

    for (size_t i = 0; i < inputPoints.size(); ++i) {
        double dist = cv::norm(inputPoints[i] - clickedPoint);
        if (dist < minDist) {
            minDist = dist;
            nearestIndex = i;
        }
    }

    return nearestIndex;
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
    
    std::tuple<cv::Mat*, std::vector<cv::Point2d>*, std::vector<cv::Point2d>*, std::string, std::string, double, int, int>* data =
        (std::tuple<cv::Mat*, std::vector<cv::Point2d>*, std::vector<cv::Point2d>*, std::string, std::string, double, int, int>*) userdata;
    cv::Mat& image = *(std::get<0>(*data));
    std::vector<cv::Point2d>& inputPoints = *(std::get<1>(*data));
    std::vector<cv::Point2d>& clickedPoints = *(std::get<2>(*data));
    std::string inputFile = std::get<3>(*data);
    std::string FileOutput = std::get<4>(*data);
    double d = std::get<5>(*data);
    int row = std::get<6>(*data);
    int col = std::get<7>(*data);

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (clickedPoints.size() < 4) {
            clickedPoints.emplace_back(x, y);
            std::cout << "Point added - position (" << x << ", " << y << ")" << std::endl;


            if (clickedPoints.size() == 4) {
                std::cout << "Order of clicked points: ";
                for (const cv::Point2d& p : clickedPoints) {
                    std::cout << "(" << p.x << ", " << p.y << ") ";
                }
                std::cout << std::endl;
                std::vector<cv::Vec3d> XYZ;
                std::vector<cv::Point2d> gridPoints;
                

                for (int i = 0; i < col; ++i) {
                    double t = static_cast<double>(i) / (col-1);
                    cv::Point2d p1 = clickedPoints[0] * (1.0 - t) + clickedPoints[3] * t;
                    cv::Point2d p2 = clickedPoints[1] * (1.0 - t) + clickedPoints[2] * t;

                    for (int j = 0; j < row; ++j) {
                        double s = static_cast<double>(j) / (row-1);
                        cv::Point2d gridPoint = p1 * (1.0 - s) + p2 * s;
                        gridPoints.push_back(gridPoint);
                        XYZ.push_back(cv::Vec3d(j*d,i*d, 0));
                    }
                }

                std::vector<cv::Point2d> gridPointsOrder;
                std::vector<cv::Point2d> Inputforbestresult = inputPoints;

                for (int i = 0; i < gridPoints.size(); ++i) {
                    int nearestIndex = findNearestInputPoint(Inputforbestresult, gridPoints[i]);
                    gridPointsOrder.push_back(Inputforbestresult[nearestIndex]);   
                    
                    Inputforbestresult.erase(Inputforbestresult.begin() + nearestIndex);
                }
                

                drawPoints(image, gridPointsOrder, cv::Scalar(0, 255, 255)); 

                std::ofstream outputFile(FileOutput, std::ios::app);
                if (outputFile.is_open()) {
                    for (size_t i = 0; i < gridPointsOrder.size(); ++i) {
                        outputFile << XYZ[i][0] << " " << XYZ[i][1] << " " << XYZ[i][2] <<" " << gridPointsOrder[i].x << " " << gridPointsOrder[i].y << std::endl;
                    }
                    outputFile.close();
                    std::cout << "Points saved to "<< FileOutput << std::endl;
                }
                else {
                    std::cerr << "Unable to open file for writing!" << std::endl;
                }

                std::ofstream file(FileOutput, std::ios::app);

                if (file.is_open()) {

                    file << "1\n";


                    file.close();

                    std::cout << "Element added successfully in "<< FileOutput << std::endl;
                }
                else {
                    std::cerr << "Error opening file." << std::endl;
                }
            }
        }

        drawArrows(image, clickedPoints);

        cv::imshow("Image with points", image);
    }    
}

int main(int argc, char** argv) {

    if (argc < 4) {
        std::cerr << "Usage: ./program <input_file.txt> <output_file.txt> <image_file.png> Dist row cols" << std::endl;
        std::cerr << "rows : X axis & cols : Y axis ont 6 comme valeur par defaut. Pour Dist la valeur est de 0.3" << std::endl;
        return -1;
    }


    cv::Mat image = cv::imread(argv[3]);
    double dist = argc > 4 ? std::stod(argv[4]) : 0.3;
    int rows = argc > 5 ? std::stoi(argv[5]) : 6; 
    int cols = argc > 5 ? std::stoi(argv[6]) : 6; 



    if (image.empty()) {
        std::cerr << "Impossible to load the image !" << std::endl;
        return -1;
    }


    std::ifstream inputFile(argv[1]);
    if (!inputFile.is_open()) {
        std::cerr << "Unable to open the input file !" << std::endl;
        return -1;
    }
    
    std::vector<cv::Point2d> clickedPoints;
    std::vector<cv::Point2d> points;
    double x, y;
    while (inputFile >> x >> y) {
        points.emplace_back(x, y);
    }
    inputFile.close();


    drawPoints(image, points, cv::Scalar(255, 0, 255));
    

    cv::putText(image, "Are the points in the right order? Press Y for yes, N for no.", cv::Point(40, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 20, 20), 2);


    cv::namedWindow("Image with points");
    cv::imshow("Image with points", image);

    
    char key = (char)cv::waitKey(0);

    if (key == 'n' || key == 'N') {
    	 
        std::cout << "Please click on the 4 corners of the calibration pattern : the first click on the origin, the second click on the corner giving the x direction, the third click on the point opposite the origin and the last click on the last corner to complete the square of the calibration pattern." << std::endl;
        cv::putText(image, "Please refer to the instructions in the command prompt", cv::Point(image.cols / 4, image.rows / 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(125, 125, 125), 2);
        std::tuple<cv::Mat*, std::vector<cv::Point2d>*, std::vector<cv::Point2d>*, std::string, std::string, double, int, int> data =
            std::make_tuple(&image, &points, &clickedPoints, argv[1], argv[2], dist, rows, cols);

        cv::setMouseCallback("Image with points", onMouse, (void*)&data);


        while (clickedPoints.size() < 4) {
            cv::imshow("Image with points", image);
            cv::waitKey(5000);
        }
        
        
    }
    else if (key == 'y' || key == 'Y') {
        cv::putText(image, "Classified ! ", cv::Point(image.cols/2, image.rows/2), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar( 255, 0, 0), 2);
        cv::imshow("Image with points", image);

        std::vector<cv::Vec3d> XYZ;

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                XYZ.push_back(cv::Vec3d(j * dist, i * dist, 0));
            }
        }
        
        std::ofstream outputFile(argv[2], std::ios::app);
        
        if (outputFile.is_open()) {
            for (size_t i = 0; i < XYZ.size(); ++i) {
                outputFile << XYZ[i][0] << " " << XYZ[i][1] << " " << XYZ[i][2] << " " << points[i].x << " " << points[i].y << std::endl;
            }
            outputFile.close();
            
            std::cout << "Points saved to " << argv[2] << std::endl;
        }
        else {
            std::cerr << "Unable to open file for writing!" << std::endl;
        }

        std::ofstream file(argv[2], std::ios::app);

        if (file.is_open()) {

            file << "1\n";


            file.close();

            std::cout << "Element added successfully in " << argv[2] << std::endl;
        }
        else {
            std::cerr << "Error opening file." << std::endl;
        }
    }


    drawPoints(image, clickedPoints, cv::Scalar(0, 0, 255)); 


    cv::imshow("Image with points", image);
    cv::waitKey(0);


    cv::destroyAllWindows();

    return 0;
}

