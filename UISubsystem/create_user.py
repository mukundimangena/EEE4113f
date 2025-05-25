# create_user.py
from models import Base, User
from database import engine, SessionLocal
from auth import hash_password

Base.metadata.create_all(bind=engine)
db = SessionLocal()

# Add new users here
users_to_add = [
    {"username": "christina", "password": "Christina123"},
   
]

for user_data in users_to_add:
    if not db.query(User).filter(User.username == user_data["username"]).first():
        new_user = User(
            username=user_data["username"],
            hashed_password=hash_password(user_data["password"])
        )
        db.add(new_user)
        print(f"User {user_data['username']} created successfully")
    else:
        print(f"User {user_data['username']} already exists")

db.commit()
db.close()