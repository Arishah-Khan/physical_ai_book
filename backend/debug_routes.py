from main import app
import json

print("All registered routes:")
for route in app.routes:
    print(f"Path: {route.path}, Methods: {getattr(route, 'methods', 'N/A')}")

print("\nDetailed route information:")
for route in app.routes:
    if hasattr(route, 'methods'):
        print(f"Route: {route.path}")
        print(f"  Methods: {route.methods}")
        print(f"  Name: {getattr(route, 'name', 'N/A')}")
        print(f"  Endpoint: {getattr(route, 'endpoint', 'N/A')}")
        print()