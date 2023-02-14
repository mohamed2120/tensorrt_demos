from wiring.wiring import wire_entities
from utils.parse_args import parse_args
from utils.camera import Camera as CameraUtil
from utils.display import open_window

if __name__ == '__main__':
    try:
        window_name='Cap_App'
        args = parse_args()
        # cls_dict = get_cls_dict(args.category_num)
        # vis = BBoxVisualization(cls_dict)
        cam =  CameraUtil(args)
        if not cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')
            
        # open_window(
        #     window_name, 'Camera TensorRT YOLO Demo',
        #     cam.img_width, cam.img_height)
        
        wire_entities(args, cam, window_name)
    except KeyboardInterrupt:
        # endprogram()
        print("got here first*************************************************")
        #release cam and destroy windows
        # cv2.destroyAllWindows()
        # cam.release()
        sys.exit(0)