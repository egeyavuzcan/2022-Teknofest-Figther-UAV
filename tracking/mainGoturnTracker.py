from tracking.goturn.logger.logger import setup_logger
from tracking.goturn.network.regressor import regressor
from tracking.goturn.tracker.tracker import tracker
from tracking.goturn.tracker.tracker_manager import tracker_manager

prototxt = 'goturn/nets/tracker.prototxt'
model = 'goturn/nets/tracker.caffemodel'
logger = setup_logger(logfile=None)


inputVideoPath = 'Input/Trim840.mp4'

objRegressor = regressor(prototxt, model, logger)
objTracker = tracker(objRegressor)
objTrackerVis = tracker_manager(objRegressor, objTracker, logger)

objTrackerVis.trackAll(inputVideoPath, save_tracking=False, show_tracking=True)
