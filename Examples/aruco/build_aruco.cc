#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

int main()
{
    // Initialize marker matrix
    Mat marker;

    // Get predefined dictionary
    auto dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_7X7_50);

    // Draw and display/save markers
    for (int i = 0; i < 14; i++)
    {
        aruco::drawMarker(dictionary, i, 200, marker);

        string windowName = "marker" + to_string(i);
        
        string path = "output/" + windowName + ".jpg"; // Change 'yourusername' to your actual username
        imwrite(path, marker);
    } 

    return 0;
}
