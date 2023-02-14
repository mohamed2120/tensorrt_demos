import cv2
import datetime
from lib.entity_types.SensorController import SensorController
from lib.event_bus.decorators import atomic


class ImageSaver(SensorController):
    rule = []
    states = []

    def __init__(self):
        super(ImageSaver, self).__init__(name='image-saver',
                                         states=ImageSaver.states, rule=ImageSaver.rule)

    def _do_saving(self, img, boxes, confs, clss):
        now = datetime.datetime.now().strftime('%d%m%Y_%H%M%S')
        box_with_low_conf = False
        for idx in range(len(boxes)):
            if confs[idx] < 0.90:
                box_with_low_conf = True
        if box_with_low_conf:
            cv2.imwrite('./auto-labeling/images/frame_' +
                        now+'.jpg', img)
            label_file = './auto-labeling/labels/frame_'+now+'.txt'
            # open file in append mode
            f = open(label_file, "a")
            for idx in range(len(boxes)):
                # current box
                bb = boxes[idx]
                # get height and width of the image
                (original_height, original_width) = img.shape[:2]
                # x represents the vertical axis while y represents the horizontal axis
                # get top left (x_min,y_min) and bottom right (x_max,y_max)
                x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
                # calculate box height
                b_height = (x_max-x_min)
                # calculate box width
                b_width = (y_max-y_min)
                # convert absolute values of top left corner of currrent box to relative by dividing by height and width respectively
                x_min /= original_height
                y_min /= original_width
                # convert absolute values of height and width of currrent box to relative by dividing by height and width respectively
                b_height /= original_height
                b_width /= original_width
                # create line consisting of those values
                line = str(int(clss[idx]))+" "+str(x_min)+" " + \
                    str(y_min)+" "+str(b_height) + \
                    " "+str(b_width)+"\n"
                # write the line to file
                f.write(line)
                # close the file
            f.close()
            print('** saved img')
        else:
            # save the image that the model doesn't detect a single box in it in non_labeled_images folder
            cv2.imwrite('./auto-labeling/non_labeled_images/frame_' +
                        now+'.jpg', img)
            print('** saved no label image')

    def _do_fake_savint(self, img, boxed, confs, clss):
        print(img, "got image to save")

    @atomic(device='image-saver')
    def handel_detector(self, event):
        meta_data = event.get('event').get('meta-data')
        self._do_saving(meta_data.get('img'), meta_data.get('boxes'), meta_data.get(
            'confs'), meta_data.get('clss'))
