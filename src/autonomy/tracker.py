import cv2
import numpy as np

# Load Yolo
net = cv2.dnn.readNet("./trained/yolo-drone.weights", "./trained/yolo-drone.cfg")
classes = ["drone"]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(3, 3))

def track_and_display( cv_image, display = False):
    height, width, channels = cv_image.shape

    blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                # Rectangle coordinates
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    print(indexes)
    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            color = colors[i]
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
            #cv2.putText(cv_image, label, (x, y + 30), font, 3, color, 3)


    cv2.imshow("Image", cv_image)
    cv2.waitKey(3)
    #cv2.destroyAllWindows()