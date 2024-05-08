# patch_simu

This POLARIS_GEM_e2 in [github](https://github.com/GEM-Illinois/POLARIS_GEM_e2) is deprecated. The project has been migrated to [gitlab](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2).

Some updates seems required to use properly the simulator. We propose here to apply the current patch.

# Installation 

```
git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
cd POLARIS_GEM_e2/
git am simu_update.patch
```

# List of updates

Please read the [last changelog](./CHANGELOG.md#Changelog)

# How to generate the patch

Do your update on your branch, commit the update , and geenrate the patch diff from the command:

```
git format-patch origin/main --stdout > simu_update.patch
```