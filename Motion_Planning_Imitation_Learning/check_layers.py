import h5py

with h5py.File('model.h5', 'r') as f:
    print(list(f.keys()))  # Print the names of the groups in the file
    # Print the names of the datasets in each group
    for group_name in f.keys():
        print(f[group_name].name)
        print(list(f[group_name].keys()))