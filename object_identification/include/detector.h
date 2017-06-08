#ifndef OBJECT_POSITIONS_DETECTOR_H
#define OBJECT_POSITIONS_DETECTOR_H

#ifdef OPENCV
#include "opencv2/highgui/highgui_c.h"
// added later
#include "opencv2/imgproc/imgproc_c.h"
#endif

//#ifdef __cplusplus
//extern "C" {
//#endif


typedef struct {
    int left;
    int right;
    int top;
    int bot;
    float prob;
    char label[1024];
} labeld_squares;

/** Get The labeled prediction scuares.
 * @param lib_path    - The library path.
 * @param cfg_file    - The content of the configuration file.
 * @param weight_file - The file on the trained module.
 * @param file_name   - The input image.
 * @param thresh      - The detection probability threshold.
 * @param detections  - The returned detection result squares.
 * @returns The detections count.
 */
int
predict_scuares(const char* lib_path,
                const char* data_cfg,
                const char* cfg_file,
                const char* weight_file,
                const char* file_name,
                float thresh, labeld_squares** detections);

int predict_scuares(const char* lib_path,
                const char* data_cfg,
                const char* cfg_file,
                const char* weight_file,
				const IplImage* mat_input,
                float thresh,
                labeld_squares** predictions);
                
int predict_scuares_org(const char* lib_path,
                const char* data_cfg,
                const char* cfg_file,
                const char* weight_file,
                const char* file_name,
                float thresh,
                labeld_squares** predictions);

//#ifdef __cplusplus
//}
//#endif

#endif
