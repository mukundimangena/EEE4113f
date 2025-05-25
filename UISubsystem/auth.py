from passlib.context import CryptContext

# Use 'django_bcrypt' or 'auto' as fallback
pwd_context = CryptContext(
    schemes=["bcrypt", "django_bcrypt"],
    deprecated="auto"
)

def verify_password(plain_password, hashed):
    return pwd_context.verify(plain_password, hashed)

def hash_password(password):
    return pwd_context.hash(password)

