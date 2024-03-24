from ultralytics import YOLO

# Load a model
# model = YOLO("args.yaml")  # build a new model from scratch
model = YOLO("best.pt")  # load a pretrained model (recommended for training)
results = model(source=0, show=True, conf=0.3, save=True, stream=True)