# Add a Notification to the database
from models import Base, Notification
from models import Base, User
from database import engine, SessionLocal


Base.metadata.create_all(bind=engine)
db = SessionLocal()

# Add new users here thsi is not an objecr
Add_Notification = [{
    "title": "New Feature",
    "content": "We have added a new feature to our app!",
    "time": "2023-10-01 12:00:00",
    "priority": "High",
    "location": "Home",
    "status": "Unread",
    "icon": "info"
}]

for notification in Add_Notification :
    if not db.query(Notification).filter(Notification.time == notification["time"]).first():
       new_notification = Notification(
            title=notification["title"],
            content=notification["content"],
            time=notification["time"],
            priority=notification["priority"],
            location=notification["location"],
            status=notification["status"],
            icon=notification["icon"]
          )
       db.add(new_notification)
       print(f"Notification {new_notification.title} created successfully")
    else:
        print(f"Notification {notification['title']} already exists")

db.commit()
db.close()