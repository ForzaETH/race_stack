from helpers.load_model import get_dotdict

model_name = "NUC1_linear" #name + tire model name

print(f"Loading model {model_name}...")
model = get_dotdict(model_name)
for name, value in model.items():
    print(str(name) + ": " + str(value))
