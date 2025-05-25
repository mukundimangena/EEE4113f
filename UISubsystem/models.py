# models.py
from sqlalchemy import Column, Integer, String, Float, DateTime
from sqlalchemy.ext.declarative import declarative_base
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

Base = declarative_base()

# SQLAlchemy Models (for database)
class User(Base):
    __tablename__ = "users"
    
    id = Column(Integer, primary_key=True, index=True)
    username = Column(String, unique=True, index=True)
    hashed_password = Column(String)

class Notification(Base):
    __tablename__ = "notifications"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    content = Column(String, nullable=False)
    time = Column(DateTime, default=datetime.utcnow)  # Changed to DateTime
    priority = Column(String, nullable=False)
    location = Column(String, nullable=False)
    status = Column(String, nullable=False)
    icon = Column(String, nullable=False)
    event_type = Column(String)  # for system logs
    device = Column(String)      # for system logs
    
class NotificationBase(BaseModel):
    title: str
    content: str
    time: Optional[str] = None  # Make optional with default None
    priority: Optional[str] = "Medium"  # Default value
    location: Optional[str] = "Unknown"  # Default value
    status: Optional[str] = "Unread"  # Default value
    icon: Optional[str] = "alert"  # Default value

class NotificationCreate(NotificationBase):
    pass

class NotificationResponse(NotificationBase):
    id: int
    
    class Config:
        from_attributes = True

class ESPTEST(Base):
    __tablename__ = "esptest"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, nullable=False)
    value = Column(String, nullable=False)
    time = Column(String, nullable=False)
    status = Column(String, nullable=False)

# SQLAlchemy model for sensor data
class SensorData(Base):
    __tablename__ = "sensor_data"
    
    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String, nullable=False)
    sensor_type = Column(String, nullable=False)
    value = Column(Float, nullable=False)  # Changed to Float for sensor readings
    timestamp = Column(DateTime, default=datetime.utcnow)
    location = Column(String, nullable=True, default="unknown")

# Pydantic Models (for API request/response validation)
class SensorReading(BaseModel):
    """Pydantic model for incoming sensor data"""
    device_id: str
    sensor_type: str
    value: float
    location: Optional[str] = "unknown"

class BatchSensorData(BaseModel):
    """Pydantic model for batch sensor data"""
    readings: List[SensorReading]

class SensorResponse(BaseModel):
    """Pydantic model for sensor data responses"""
    id: int
    device_id: str
    sensor_type: str
    value: float
    timestamp: str
    location: str
    
    class Config:
        from_attributes = True  # This allows conversion from SQLAlchemy models

class APIResponse(BaseModel):
    """Pydantic model for API responses"""
    status: str
    message: str
    id: Optional[int] = None
    ids: Optional[List[int]] = None

class DeviceInfo(BaseModel):
    """Pydantic model for device information"""
    device_id: str
    location: str
    total_readings: int
    last_seen: str

class SensorDataResponse(BaseModel):
    """Pydantic model for sensor data responses (matches SensorData SQLAlchemy model)"""
    id: int
    device_id: str
    sensor_type: str
    value: float
    timestamp: datetime
    location: Optional[str] = "unknown"

    class Config:
        from_attributes = True  # Allows conversion from SQLAlchemy model