#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import json
import os
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
import sqlite3
from datetime import datetime
import math

import torch
from PIL import Image
import open_clip
import cv_bridge
import cv2



def check_clip_availability():
    """Check if CLIP dependencies are available"""
    try:
        import torch
        from PIL import Image
        import open_clip
        return True
    except ImportError as e:
        return False

CLIP_AVAILABLE = False

class OfficeWorldClassifier:
    """LIDAR-based room classification for office environment"""
    
    def __init__(self):
        # Office-specific room rules based on the world layout
        self.room_rules = {
            'reception': {
                'min_area': 20.0, 'max_area': 60.0,
                'furniture_density_range': (0.3, 0.6),
                'near_entrance': True
            },
            'open_office': {
                'min_area': 100.0, 'max_area': 300.0,
                'high_furniture_density': 0.4,
                'cubicle_pattern': True
            },
            'meeting_room_small': {
                'min_area': 15.0, 'max_area': 35.0,
                'central_table': True,
                'chair_arrangement': 'around_table'
            },
            'meeting_room_large': {
                'min_area': 30.0, 'max_area': 80.0,
                'central_table': True,
                'conference_setup': True
            },
            'office_small': {
                'min_area': 8.0, 'max_area': 20.0,
                'desk_and_chair': True,
                'private_space': True
            },
            'office_large': {
                'min_area': 20.0, 'max_area': 50.0,
                'executive_furniture': True,
                'spacious_layout': True
            },
            'break_room': {
                'min_area': 15.0, 'max_area': 40.0,
                'kitchen_appliances': True,
                'casual_seating': True
            },
            'bathroom': {
                'min_area': 4.0, 'max_area': 15.0,
                'bathroom_fixtures': True,
                'small_enclosed': True
            },
            'corridor': {
                'min_aspect_ratio': 3.0,
                'connecting_space': True,
                'low_furniture': True
            },
            'storage_room': {
                'min_area': 5.0, 'max_area': 25.0,
                'high_furniture_density': 0.7,
                'boxes_and_shelves': True
            },
            'server_room': {
                'min_area': 10.0, 'max_area': 30.0,
                'equipment_density': 0.8,
                'technical_space': True
            }
        }
    
    def analyze_scan(self, laser_scan):
        """Extract features for office environment"""
        ranges = np.array(laser_scan.ranges)
        angles = np.linspace(laser_scan.angle_min, laser_scan.angle_max, len(ranges))
        
        # Filter valid readings
        valid_mask = (ranges >= laser_scan.range_min) & (ranges <= laser_scan.range_max) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) < 10:
            return None
        
        # Convert to cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # Calculate office-specific features
        x_range = np.ptp(x) if len(x) > 0 else 0
        y_range = np.ptp(y) if len(y) > 0 else 0
        
        area = x_range * y_range
        length = max(x_range, y_range)
        width = min(x_range, y_range)
        aspect_ratio = length / max(width, 0.1)
        
        # Furniture density (obstacles closer than walls)
        wall_distance = np.percentile(valid_ranges, 85)
        furniture_threshold = wall_distance * 0.6
        furniture_density = np.sum(valid_ranges < furniture_threshold) / len(valid_ranges)
        
        # Detect patterns (simplified)
        cubicle_pattern = self._detect_cubicle_pattern(x, y)
        central_furniture = self._detect_central_furniture(x, y)
        
        return {
            'area': area,
            'length': length,
            'width': width,
            'aspect_ratio': aspect_ratio,
            'furniture_density': furniture_density,
            'cubicle_pattern': cubicle_pattern,
            'central_furniture': central_furniture,
            'perimeter': 2 * (x_range + y_range)
        }
    
    def _detect_cubicle_pattern(self, x, y):
        """Detect if space has cubicle-like furniture arrangement"""
        if len(x) < 20:
            return False
        
        # Look for regular spacing in furniture
        furniture_points = np.column_stack([x, y])
        # Simplified: check for multiple small clusters
        from sklearn.cluster import DBSCAN
        try:
            clustering = DBSCAN(eps=1.0, min_samples=3).fit(furniture_points)
            n_clusters = len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
            return n_clusters >= 4  # Multiple furniture clusters suggest cubicles
        except:
            return False
    
    def _detect_central_furniture(self, x, y):
        """Detect if there's central furniture (meeting table, etc.)"""
        if len(x) < 10:
            return False
        
        center = np.array([np.mean(x), np.mean(y)])
        distances_to_center = np.linalg.norm(np.column_stack([x, y]) - center, axis=1)
        
        # Check if there are objects near the center
        center_threshold = np.percentile(distances_to_center, 30)
        central_objects = np.sum(distances_to_center < center_threshold)
        
        return central_objects > len(x) * 0.15  # At least 15% of objects near center
    
    def classify_room(self, laser_scan):
        """Classify office room type"""
        features = self.analyze_scan(laser_scan)
        if features is None:
            return 'unknown', 0.1, features
        
        scores = {}
        
        for room_type, rules in self.room_rules.items():
            score = 0.0
            max_score = 0.0
            
            # Area checks
            if 'min_area' in rules:
                max_score += 1
                if features['area'] >= rules['min_area']:
                    score += 1
            if 'max_area' in rules:
                max_score += 1
                if features['area'] <= rules['max_area']:
                    score += 1
            
            # Aspect ratio for corridors
            if 'min_aspect_ratio' in rules:
                max_score += 1
                if features['aspect_ratio'] >= rules['min_aspect_ratio']:
                    score += 1
            
            # Furniture density checks
            if 'high_furniture_density' in rules:
                max_score += 1
                if features['furniture_density'] >= rules['high_furniture_density']:
                    score += 1
            
            if 'furniture_density_range' in rules:
                max_score += 1
                min_dens, max_dens = rules['furniture_density_range']
                if min_dens <= features['furniture_density'] <= max_dens:
                    score += 1
            
            # Pattern detection
            if 'cubicle_pattern' in rules and rules['cubicle_pattern']:
                max_score += 1
                if features['cubicle_pattern']:
                    score += 1
            
            if 'central_table' in rules and rules['central_table']:
                max_score += 1
                if features['central_furniture']:
                    score += 1
            
            # Normalize score
            if max_score > 0:
                scores[room_type] = score / max_score
            else:
                scores[room_type] = 0.0
        
        if not scores:
            return 'unknown', 0.1, features
        
        best_room = max(scores.items(), key=lambda x: x[1])
        return best_room[0], best_room[1], features

class CLIPOfficeClassifier:
    """CLIP classifier for office environments"""
    
    def __init__(self, model_name="ViT-B-32"):
        
        self.torch = torch
        self.open_clip = open_clip
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {self.device}")
        
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(model_name, pretrained='laion2b_s34b_b79k')
        self.model.eval()  # model in train mode by default, impacts some models with BatchNorm or stochastic depth active
        self.tokenizer = open_clip.get_tokenizer(model_name)
        
        # Office-specific room labels
        self.room_labels = [
            "modern office reception area with desk and seating",
            "open office space with cubicles and workstations",
            "small meeting room with conference table and chairs",
            "large conference room with presentation equipment",
            "private office with desk computer and chair",
            "executive office with large desk and bookshelf",
            "office break room with kitchen appliances and tables",
            "office bathroom with sink and toilet",
            "office corridor hallway with doors",
            "storage room with shelves and office supplies",
            "server room with computer equipment and racks"
        ]
        
        self.label_mapping = {
            "modern office reception area with desk and seating": "reception",
            "open office space with cubicles and workstations": "open_office",
            "small meeting room with conference table and chairs": "meeting_room_small",
            "large conference room with presentation equipment": "meeting_room_large",
            "private office with desk computer and chair": "office_small",
            "executive office with large desk and bookshelf": "office_large",
            "office break room with kitchen appliances and tables": "break_room",
            "office bathroom with sink and toilet": "bathroom",
            "office corridor hallway with doors": "corridor",
            "storage room with shelves and office supplies": "storage_room",
            "server room with computer equipment and racks": "server_room"
        }
    
    def classify_image(self, cv_image):
        """Classify office image using CLIP"""
        try:
            # Import cv2 locally
            import cv2
            
            # Convert and preprocess
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb_image)
            
            image = self.preprocess(pil_image).unsqueeze(0)
            text = self.tokenizer(self.room_labels)
            
            with self.torch.no_grad(), self.torch.autocast("cuda" if self.device == "cuda" else "cpu"):
                image_features = self.model.encode_image(image)
                text_features = self.model.encode_text(text)
                image_features /= image_features.norm(dim=-1, keepdim=True)
                text_features /= text_features.norm(dim=-1, keepdim=True)
                
                text_probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)
                probs = text_probs.cpu().numpy()[0]
            
            # Get top predictions
            top_indices = np.argsort(probs)[::-1][:3]
            results = []
            
            for idx in top_indices:
                detailed_label = self.room_labels[idx]
                simple_label = self.label_mapping.get(detailed_label, detailed_label)
                confidence = float(probs[idx])
                results.append((simple_label, confidence, detailed_label))
            
            return results[0][0], results[0][1], results
            
        except Exception as e:
            print(f"CLIP classification error: {e}")
            return 'unknown', 0.1, []

class OfficeSemanticNode(Node):
    def __init__(self):
        super().__init__('office_semantic_node')
        
        # Parameters
        self.declare_parameter('capture_interval', 5.0)
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('lidar_weight', 0.6)
        self.declare_parameter('vision_weight', 0.4)
        self.declare_parameter('use_clip', True)
        
        # Get parameters
        self.capture_interval = self.get_parameter('capture_interval').get_parameter_value().double_value
        self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
        self.lidar_weight = self.get_parameter('lidar_weight').get_parameter_value().double_value
        self.vision_weight = self.get_parameter('vision_weight').get_parameter_value().double_value
        self.use_clip = self.get_parameter('use_clip').get_parameter_value().bool_value
        
        # Publishers
        self.semantic_pub = self.create_publisher(String, '/semantic_labels', 10)
        self.analysis_pub = self.create_publisher(String, '/semantic_analysis', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.query_sub = self.create_subscription(
            String, '/semantic_query', self.query_callback, 10)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize classifiers
        self.lidar_classifier = OfficeWorldClassifier()
        
        # Try to initialize CLIP
        self.vision_classifier = None
        if self.use_clip:
            CLIP_AVAILABLE = check_clip_availability()
            if CLIP_AVAILABLE:
                try:
                    self.vision_classifier = CLIPOfficeClassifier()
                    self.get_logger().info("🏢 Office CLIP classifier initialized!")
                except Exception as e:
                    self.get_logger().error(f"Failed to initialize CLIP: {e}")
                    self.use_clip = False
            else:
                self.get_logger().warn("CLIP dependencies not available, disabling vision classification")
                self.use_clip = False
        
        # CV Bridge (only if CLIP available)
        if self.use_clip:
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
                self.image_sub = self.create_subscription(
                    Image, '/camera/image_raw', self.image_callback, 10)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize CV Bridge: {e}")
                self.use_clip = False
        
        # Database setup
        self.setup_database()
        
        # State
        self.last_capture_time = 0.0
        self.latest_scan = None
        self.latest_image = None
        self.analysis_count = 0
        
        mode = "LIDAR + CLIP" if self.use_clip else "LIDAR-only"
        self.get_logger().info(f"🏢 Office semantic mapping initialized ({mode})")
    
    def setup_database(self):
        """Initialize database with office schema"""
        db_path = os.path.expanduser('~/office_semantic_map.db')
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        
        cursor = self.conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS office_locations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL NOT NULL,
                y REAL NOT NULL,
                timestamp TEXT NOT NULL,
                room_type TEXT NOT NULL,
                confidence REAL NOT NULL,
                lidar_features TEXT,
                vision_results TEXT,
                fusion_method TEXT,
                area REAL,
                furniture_density REAL
            )
        ''')
        self.conn.commit()
        
        self.get_logger().info(f"🏢 Office database initialized at {db_path}")
    
    def get_current_pose(self):
        """Get current robot pose"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception as e:
            return None, None
    
    def scan_callback(self, msg):
        """Store latest LIDAR scan"""
        self.latest_scan = msg
        self.try_analysis()
    
    def image_callback(self, msg):
        """Store latest camera image"""
        if not self.use_clip:
            return
            
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def try_analysis(self):
        """Perform analysis when data is available"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Rate limiting
        if current_time - self.last_capture_time < self.capture_interval:
            return
        
        if self.latest_scan is None:
            return
        
        if self.use_clip and self.latest_image is None:
            return
        
        self.last_capture_time = current_time
        
        # Get current pose
        x, y = self.get_current_pose()
        if x is None or y is None:
            return
        
        self.analysis_count += 1
        
        # LIDAR analysis
        lidar_label, lidar_confidence, lidar_features = self.lidar_classifier.classify_room(self.latest_scan)
        
        # Vision analysis
        vision_label, vision_confidence, vision_results = None, 0.0, []
        if self.use_clip and self.vision_classifier is not None:
            vision_label, vision_confidence, vision_results = self.vision_classifier.classify_image(self.latest_image)
        
        # Fusion
        final_label, final_confidence, fusion_method = self.fuse_classifications(
            lidar_label, lidar_confidence, vision_label, vision_confidence)
        
        # Store results
        if final_confidence >= self.min_confidence:
            self.store_office_location(
                x, y, final_label, final_confidence, 
                lidar_features, vision_results, fusion_method)
            
            # Publish results
            semantic_msg = String()
            semantic_msg.data = f"🏢 {final_label} at ({x:.2f}, {y:.2f}) [conf: {final_confidence:.2f}]"
            self.semantic_pub.publish(semantic_msg)
            
            self.get_logger().info(f"🏢 Detected: {final_label} at ({x:.2f}, {y:.2f})")
    
    def fuse_classifications(self, lidar_label, lidar_conf, vision_label, vision_conf):
        """Fuse LIDAR and vision classifications"""
        if vision_label is None:
            return lidar_label, lidar_conf, 'lidar_only'
        
        if lidar_label == vision_label:
            boosted_confidence = min(
                self.lidar_weight * lidar_conf + self.vision_weight * vision_conf + 0.1, 
                1.0
            )
            return lidar_label, boosted_confidence, 'agreement_boost'
        else:
            weighted_lidar = self.lidar_weight * lidar_conf
            weighted_vision = self.vision_weight * vision_conf
            
            if weighted_lidar > weighted_vision:
                return lidar_label, weighted_lidar * 0.8, 'lidar_dominant'
            else:
                return vision_label, weighted_vision * 0.8, 'vision_dominant'
    
    def store_office_location(self, x, y, room_type, confidence, lidar_features, vision_results, fusion_method):
        """Store office location in database"""
        timestamp = datetime.now().isoformat()
        
        # Extract additional features for office analysis
        area = lidar_features.get('area', 0) if lidar_features else 0
        furniture_density = lidar_features.get('furniture_density', 0) if lidar_features else 0
        
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO office_locations 
            (x, y, timestamp, room_type, confidence, lidar_features, vision_results, fusion_method, area, furniture_density)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (x, y, timestamp, room_type, confidence,
              json.dumps(lidar_features, default=str),
              json.dumps(vision_results, default=str),
              fusion_method, area, furniture_density))
        self.conn.commit()
    
    def query_callback(self, msg):
        """Handle office semantic queries"""
        query = msg.data.lower().strip()
        self.get_logger().info(f"🔍 Office query: '{query}'")
        
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT x, y, room_type, confidence
            FROM office_locations 
            WHERE room_type LIKE ? 
            ORDER BY confidence DESC
            LIMIT 5
        ''', (f'%{query}%',))
        
        results = cursor.fetchall()
        
        if results:
            best_result = results[0]
            x, y, room_type, confidence = best_result
            
            response = String()
            response.data = f"🏢 Found {room_type} at ({x:.2f}, {y:.2f}) [confidence: {confidence:.2f}]"
            self.semantic_pub.publish(response)
            
            self.get_logger().info(f"✅ {response.data}")
        else:
            response = String()
            response.data = f"❌ No office rooms found matching '{query}'"
            self.semantic_pub.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = OfficeSemanticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'conn'):
            node.conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()