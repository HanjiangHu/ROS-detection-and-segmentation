#ifndef CLASSIFIER_H
#define CLASSIFIER_H
extern "C"
{
    #include <darknet.h>
}
/**
 * 返回结构体
 * */
struct detection{
    int num;
    float **probs;
    char **names;
    int classes;
    box *pos;
    int **selected;//选中的序号
};
detection test_result ;

    void free_ptrs1(float **ptrs, int n)
{
    int i;
    for(i = 0; i < n; ++i) free(ptrs[i]);
    free(ptrs);
}

/*节点发布*/
detection detector1(char *datacfg, char *cfgfile, char *weightfile, char *filename, float thresh, float hier_thresh, char *outfile, int fullscreen)
{
    list *options = read_data_cfg(datacfg);
    char *name_list = option_find_str(options, "names", "darknet/data/coco.names");
    char path[50] = "/home/huhanjiang/catkin_ws/src/ros_seg/";
    char **names = get_labels(strcat(path,name_list));
    image **alphabet = load_alphabet();
    network *net = load_network(cfgfile, weightfile, 0);
    set_batch_network(net, 1);
    srand(2222222);
    double time;
    char buff[256];
    char *input = buff;
    int j;
    float nms=.3;
    
        if(filename){
            strncpy(input, filename, 256);
        } else {
            printf("Enter Image Path: ");
            fflush(stdout);
            input = fgets(input, 256, stdin);
            if(!input) return test_result;
            strtok(input, "\n");
        }
        image im = load_image_color(input,0,0);
        image sized = letterbox_image(im, net->w, net->h);
        
        layer l = net->layers[net->n-1];

        box *boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
        float **probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
        for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes + 1, sizeof(float *));
        float **masks = 0;
        if (l.coords > 4){
            masks = (float **)calloc(l.w*l.h*l.n, sizeof(float*));
            for(j = 0; j < l.w*l.h*l.n; ++j) masks[j] = (float *)calloc(l.coords-4, sizeof(float *));
        }

        float *X = sized.data;
        network_predict(net, X);
        //计时
        /*time=what_time_is_it_now();
        printf("%s: Predicted in %f seconds.\n", input, what_time_is_it_now()-time);*/
        get_region_boxes(l, im.w, im.h, net->w, net->h, thresh, probs, boxes, masks, 0, 0, hier_thresh, 1);
        if (nms) do_nms_sort(boxes, probs, l.w*l.h*l.n, l.classes, nms);
        test_result.selected = draw_detections(im, l.w*l.h*l.n, thresh, boxes, probs, masks, names, alphabet, l.classes);//获得目标序号
        test_result.num = l.w*l.h*l.n;
        test_result.names = names;
        test_result.probs = probs;
        test_result.classes = l.classes;
        test_result.pos = boxes;
        if(outfile){
            save_image(im, outfile);
        }
        else{
            save_image(im, "predictions");
    /*#ifdef OPENCV
            cvNamedWindow("predictions", CV_WINDOW_NORMAL); 
            if(fullscreen){
                cvSetWindowProperty("predictions", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
            }
            show_image(im, "predictions");
            cvWaitKey(0);
            cvDestroyAllWindows();
    #endif*/
        }
        free_image(im);
        free_image(sized);
        free(boxes);
        free_ptrs1((float **)probs, l.w*l.h*l.n);
    return test_result;
}
#endif
