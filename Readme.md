MAO Manita - M2 ISICG - Semestre Automne
Mini projet dans le cadre de l'UE Modélisation et animation, partie B. Crespin.

# Description
Voici une implémentation simple de l'article "Stable Cosserat Rod".
Le code principale à lancer est CosseratRodSim.pde. Il y a aura une corde d'afficher et une petite inferface permettant de changer les coefficients k_ss ("stretching" et "shearing"), k_bt ("bending" et "twisting"), la gravité, DT (le pas de temps) et le nombre de points d'une corde. 

# Classes
- Vec3 : opérateurs sur des vecteurs 3D
- Quat : opérateurs sur des quaternions
- Segment : une corde est composée de plusieurs segments. Définitions des différents paramètres importantes pour la simulation.
- Rod : liste de segments. Contient les deux algorithmes principaux du papier "Stable Cosserat Rod".
- World : permet d'initialiser la corde. COntient les algorithmes de dessins des points et segments. 
- HScrollbar : outils pour l'interface de slider pour changer les paramètres.
- CosseratRodSim : code principale, permettant à chaque pas de temps d'appeller ```rod.step(dt)```, également d'afficher l'interface et de mettre à jour les différents paramètres. 


# Référence

```@inproceedings{Hsu2025,
    author = {Jerry Hsu, Tongtong Wang, Kui Wu, and Cem Yuksel},
    title = {Stable Cosserat Rods},
    year = {2025},
    isbn = {9798400715402},
    publisher = {Association for Computing Machinery},
    address = {New York, NY, USA},
    url = {https://doi.org/10.1145/3721238.3730618},
    doi = {10.1145/3721238.3730618},
    numpages = {10},
    keywords = {Cosserat Rods, Thin Elastic Rods},
    location = {Vancouver, BC, Canada},
    series = {SIGGRAPH ’25}
} ```