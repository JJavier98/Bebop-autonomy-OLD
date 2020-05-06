import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
import itertools
from kalman_filter2 import KalmanBoxTracker


class TrackState:

    Detected = 1
    Confirmed = 2
    Deleted = 3


class Track:

    def __init__(self, id, bbox, centroid_coor, feature):
        self.id = id
        self.bbox = bbox
        self.centroid_coor = centroid_coor
        self.kalman_filter = KalmanBoxTracker(convert_bbox_to_coor_bbox(bbox, centroid_coor))
        self.age = 0
        self.total_visible_count = 0
        self.consecutive_invisible_count = 0
        self.features = []
        self.features.append(feature)
        self.state = TrackState.Detected

    def is_detected(self):
        return self.state == TrackState.Detected

    def is_confirmed(self):
        return self.state == TrackState.Confirmed

    def is_deleted(self):
        return self.state == TrackState.Deleted

    def predict(self):
        return self.kalman_filter.predict()

    def update(self, coor_bbox):
        self.kalman_filter.update(coor_bbox)


class ObjTracker:

    def __init__(self, track_id_count=0, max_undetected_frames=60, min_visible_frames=30):
        self.tracks = []
        self.track_id_count = track_id_count
        self.max_undetected_frames = max_undetected_frames
        self.dist_unassignment_cost = 20
        self.feature_unassignment_cost = 0.6
        self.min_visible_frames = min_visible_frames
        self.feature_dict = {}
        self.dict_size = 10

    def update(self, detections, centroids, features):
        # Match entre tracks y detecciones
        assignments, unmatched_tracks, unmatched_detections = self.match_detections_to_track(centroids, features)
        self.update_tracks(assignments, unmatched_tracks, detections, centroids, features)
        self.remove_tracks(unmatched_tracks)
        self.add_new_tracks(unmatched_detections, detections, centroids, features)
        self.update_features_dict()

    def update_features_dict(self):

        features = []
        indexes = []
        active_tracks = []

        for track in self.tracks:
            if not track.is_confirmed(): continue
            active_tracks.append(track.id)
            features += track.features
            indexes += [track.id for _ in track.features]
            track.features = []

        for feature, target in zip(features, indexes):
            self.feature_dict.setdefault(target, []).append(feature)
            if len(self.feature_dict[target]) > self.dict_size:
                self.feature_dict[target] = self.feature_dict[target][-self.dict_size:]
        
        self.feature_dict = {k: self.feature_dict[k] for k in active_tracks}

    def match_detections_to_track(self, centroids, features):

        confirmed_track_indexes = [i for i, track in enumerate(self.tracks) if track.is_confirmed()]
        unconfirmed_track_indexes = [i for i, track in enumerate(self.tracks) if not track.is_confirmed()]

        assignments_1, unmatched_tracks_1, unmatched_detections_1 = self.assign_predictions_by_appearance(features, confirmed_track_indexes)

        tracks_to_evaluate = unconfirmed_track_indexes + list(unmatched_tracks_1)
        assignments_2, unmatched_tracks_2, unmatched_detections_2 = self.assign_predictions_to_tracks(centroids, tracks_to_evaluate, unmatched_detections_1)

        assignments = list(assignments_1) + list(assignments_2)

        return assignments, unmatched_tracks_2, unmatched_detections_2

    def calculate_cost_of_appearance_assigment(self, features, track_indexes, distance='euclidean'):

        n_features = len(features)
        n_tracks = len(track_indexes)
        cost = np.zeros((n_tracks, n_features))

        for i, index in enumerate(track_indexes):
            val = cdist(self.feature_dict[index], features, distance)
            cost[i,:] = val.min(axis=0)

        return cost

    def create_tracks(self, detections, centroids, features):
        for detection, centroid, feature in itertools.zip_longest(detections, centroids, features):
            track = Track(id=self.track_id_count, bbox=detection, centroid_coor=centroid, feature=feature)
            self.track_id_count += 1
            self.tracks.append(track)

    def calculate_cost_of_centroid_assigment(self, detection_centroids, tracks_to_evaluate, distance='euclidean'):
        n_tracks = len(self.tracks)
        n_detections = len(detection_centroids)
        cost = np.zeros((n_tracks, n_detections))

        track_centroids = [self.tracks[i].centroid_coor for i in tracks_to_evaluate]
        cost = cdist(np.array(track_centroids), np.array(detection_centroids), distance)

        return cost

    def assign_predictions_by_appearance(self, features, tracks_to_evaluate):

        track_ids = [self.tracks[i].id for i in tracks_to_evaluate]

        track_indices = np.arange(len(tracks_to_evaluate))
        detection_indices = np.arange(len(features))

        if len(detection_indices) == 0 or len(track_indices) == 0:
            return [], track_indices, detection_indices  # Nothing to match.

        cost = self.calculate_cost_of_appearance_assigment(features, track_ids)

        rows_id, cols_id = linear_sum_assignment(cost)

        matched_idx = np.array([[tracks_to_evaluate[row], col] for row, col in zip(rows_id, cols_id)], dtype=int)
        matched_idx = np.reshape(matched_idx, (-1, 2))

        unmatched_trackers, unmatched_detections = [], []
        for t in tracks_to_evaluate:
            if t not in matched_idx[:, 0]:
                unmatched_trackers.append(t)
        for i, f in enumerate(features):
            if i not in matched_idx[:, 1]:
                unmatched_detections.append(i)

        matches = []
        for row, col in zip(rows_id, cols_id):
            track_id = tracks_to_evaluate[row]
            if (cost[row, col] > self.feature_unassignment_cost):
                unmatched_detections.append(col)
                unmatched_trackers.append(track_id)
            else:
                matches.append((track_id, col))

        if (len(matches) == 0):
            matches = np.empty((0, 2), dtype=int)

        return matches, unmatched_trackers, unmatched_detections


    def assign_predictions_to_tracks(self, centroids, tracks_to_evaluate, unmatched_detections):

        track_indices = np.arange(len(tracks_to_evaluate))
        detection_indices = np.arange(len(unmatched_detections))

        if len(detection_indices) == 0 or len(track_indices) == 0:
            return [], track_indices, detection_indices  # Nothing to match.

        unmatched_centroids = [centroids[i] for i in unmatched_detections]

        cost = self.calculate_cost_of_centroid_assigment(unmatched_centroids, tracks_to_evaluate)

        rows_id, cols_id = linear_sum_assignment(cost)
        matched_idx = np.array([[tracks_to_evaluate[row], unmatched_detections[col]] for row, col in zip(rows_id, cols_id)], dtype=int)
        matched_idx = np.reshape(matched_idx, (-1, 2))

        unmatched_trackers, unmatched_detections_a = [], []
        for t in tracks_to_evaluate:
            if t not in matched_idx[:, 0]:
                unmatched_trackers.append(t)
        for i in unmatched_detections:
            if i not in matched_idx[:, 1]:
                unmatched_detections_a.append(i)

        matches = []
        for row, col in zip(rows_id, cols_id):
            track_id = tracks_to_evaluate[row]
            detection_id = unmatched_detections[col]
            if (cost[row, col] > self.dist_unassignment_cost):
                unmatched_detections_a.append(detection_id)
                unmatched_trackers.append(track_id)
            else:
                matches.append((track_id, detection_id))

        if (len(matches) == 0):
            matches = np.empty((0, 2), dtype=int)

        return matches, np.array(unmatched_trackers), np.array(unmatched_detections_a)

    def remove_tracks(self, unmatched_tracks_ids):
        for track_id in unmatched_tracks_ids:
            if self.tracks[track_id].consecutive_invisible_count >= self.max_undetected_frames:
                self.tracks[track_id].state = TrackState.Deleted

    def add_new_tracks(self, unmatched_detections, detections, centroids, features):
        for unmatched_detection in unmatched_detections:
            track = Track(id=self.track_id_count, bbox=detections[unmatched_detection], centroid_coor=centroids[unmatched_detection], feature=features[unmatched_detection])
            self.track_id_count += 1
            self.tracks.append(track)

    def update_tracks(self, matches, unmatched_tracks, detections, centroids, features):
        for unmatched_track in unmatched_tracks:
            coor_bbox = self.tracks[unmatched_track].predict()
            self.tracks[unmatched_track].bbox, self.tracks[
                unmatched_track].centroid_coor = self.convert_coor_bbox_to_bbox(coor_bbox)
            self.tracks[unmatched_track].consecutive_invisible_count += 1
        for match in matches:
            coor_bbox = self.tracks[match[0]].predict()
            self.tracks[match[0]].bbox, self.tracks[
                match[0]].centroid_coor = self.convert_coor_bbox_to_bbox(coor_bbox)
            self.tracks[match[0]].update(convert_bbox_to_coor_bbox(detections[match[1]], centroids[match[1]]))
            self.tracks[match[0]].consecutive_invisible_count = 0
            self.tracks[match[0]].total_visible_count += 1
            if self.tracks[match[0]].state == TrackState.Detected and self.tracks[match[0]].total_visible_count >= self.min_visible_frames:
                self.tracks[match[0]].state = TrackState.Confirmed
            self.tracks[match[0]].features.append(features[match[1]])

    def get_confirmed_tracks(self):
        tracks = []
        for track in self.tracks:
            if track.state == TrackState.Confirmed:
                tracks.append(track)
        return tracks

    def convert_coor_bbox_to_bbox(self, coor_bbox):
        x, y, s, r = coor_bbox
        w = np.sqrt(s * r)
        h = s / w
        x_img = int(round(x - w/2.))
        y_img = int(round(y - h/2.))
        return np.array([x_img, y_img, w, h]), np.array([x, y])

def convert_bbox_to_coor_bbox(bbox, centroid_coor):
    x, y = centroid_coor
    _, _, w, h = bbox
    s = w * h  # scale is just area
    r = w / float(h)
    return np.array([x, y, s, r]).reshape((4,1))