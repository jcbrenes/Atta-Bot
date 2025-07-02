# -*- coding: utf-8 -*-
"""
Created on Tue Apr  9 12:01:46 2024

@author: ccalderon
"""
import pandas as pd
import csv
import numpy as np
import json
import matplotlib.pyplot as plt
import networkx as nx
import NSGA_II4MOSPP
import MOEAD4MOSPP

def Nodo(i,j,F,C):
  return i*C+j

def VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoSuperiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j+1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i;
    s = j+1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j+1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoInferiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i+1;
    s = j-1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod


def VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i;
    s = j-1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20;
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def VecinoSuperiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P):
    r = i-1;
    s = j-1;
    k = Nodo(r,s,F,C);
    if Data[r][s] != 0:
        A[n][k] = 1;
        D[n][k] = 20*np.sqrt(2);
        P[n][k] = (Data_P[i][j]+Data_P[r][s])/2;
        Nod.setdefault(k,[D[n][k],P[n][k]]);
    return A,D,P,Nod

def GenData_ADPDic(DataFile,DataDang,File_A='A.csv',File_D='D.csv',File_P='P.csv',File_Dict='Dic.json'):
    
    Data = pd.read_csv(DataFile,sep=',',comment='#',header=None).values; #Cargar matriz de datos
    Data_P = pd.read_csv(DataDang,sep=',',comment='#',header=None).values; #Cargar matriz peligro
    
    F = len(Data); #Cantidad de filas
    C = len(Data[1]); #Cantidad de columnas
    A = np.zeros((F*C,F*C)); #Matriz de adyacencia
    D = np.zeros((F*C,F*C)); #Matriz de distancias
    P = np.zeros((F*C,F*C)); #Matriz de peligrosidad
    Net = {}; #Crea el diccionario vacio
    Cob = 0;
    
    for i in range(F):
        for j in range(C):
            if Data[i][j] != 0:
                n = Nodo(i,j,F,C); #Nodo principal en análisis
                Cob = Cob+1;
                Nod = {}; #Crea el primer nodo vacio (subdiccionario)
                if i==0:
                    if j==0: #Esquina superior izquierda
                        [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    elif j==(C-1): #Esquina superior derecha
                        [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    else: #Borde superior
                        [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoInferiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                elif i==(F-1):
                    if j == 0: #Esquina inferior izquierda
                        [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    elif j == (C-1): #Esquina inferior derecha
                        [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    else: #Borde inferior
                        [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoSuperiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                        [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                elif j == 0 and i>0 and i<(F-1): #Borde ziquierdo
                    [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                elif j ==(C-1) and i>0 and i<(F-1): #Borde derecho
                    [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                else: #Elementos del centro
                    [A,D,P,Nod] = VecinoSuperiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoSuperiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorIzquierdo(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferior(F,C,i,j,n,A,D,P,Nod,Data,Data_P);
                    [A,D,P,Nod] = VecinoInferiorDerecho(F,C,i,j,n,A,D,P,Nod,Data,Data_P); 
                Net = {**Net, n:Nod};
                Nod = {};
            else:
                n = Nodo(i,j,F,C); #Nodo principal en análisis
                Nod = {}; #Crea el primer nodo vacio (subdiccionario)
                Net = {**Net, n:Nod};
    
    with open(File_A, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(A)
        
    with open(File_D, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(D)
        
    with open(File_P, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(P)
        
    with open(File_Dict,'w') as f:
        json.dump(Net, f, sort_keys=True)
        
    with open(File_P, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(P)
        
    return A,D,P,Net,F,C,Cob
  

def Graph_Opt(adjacency_matrix,Dict,F,C,DataDang,save_graph,FileName='Graph.png',show_graph=False,Alg=0):
    #https://stackoverflow.com/questions/29572623/plot-networkx-graph-from-adjacency-matrix-in-csv-file
    #https://cienciadedatos.net/documentos/pygml01-introduccion-grafos-redes-python
    #https://networkx.org/documentation/stable/reference/generated/networkx.drawing.layout.spring_layout.html
    #Genera el grafo
    rows, cols = np.where(adjacency_matrix == 1)
    edges = zip(rows.tolist(), cols.tolist())
    gr = nx.Graph()
    gr.add_edges_from(edges)
    fixed_positions = {n: (n-(n//C)*C+0.5,F-n//C-0.5) for n in gr.nodes}
    nx.draw_networkx(gr, node_size=3, node_color='black', edge_color='black',alpha=0.4, pos=fixed_positions,with_labels=False) #labels={n: n for n in gr.nodes}   
    #Determina la ruta óptima
    source_node = np.min(rows)
    destination_node = np.max(rows)
    if Alg == 0:
        path_Opt=NSGA_II4MOSPP.main(Dict,source_node,destination_node);#Genera tres posibles caminos
    elif Alg ==1:
        path_Opt=MOEAD4MOSPP.main(Dict,source_node,destination_node,20);
    p=path_Opt[0];
    nodes_Opt=list(p['path']);
    edges_Opt = [(nodes_Opt[i],nodes_Opt[i+1]) for i in range(len(nodes_Opt)-1)]
    sub_Opt = nx.Graph()
    sub_Opt.add_nodes_from(nodes_Opt)
    sub_Opt.add_edges_from(edges_Opt)
    fixed_position_opt = {n: (n-(n//C)*C+0.5,F-n//C-0.5) for n in nodes_Opt}
    nx.draw_networkx(sub_Opt, node_size=4, node_color='r', width=3, edge_color='r', pos=fixed_position_opt, with_labels=False)
    #Presenta el grafo completo, la ruta óptima y el mapeo de la peligrosidad
    Data_P = pd.read_csv(DataDang,sep=',',comment='#',header=None).values;
    dim = len(Data_P);
    Data_P = np.insert(Data_P, Data_P.shape[0], Data_P[dim-1], 0);
    Data_P = np.insert(Data_P, Data_P.shape[1], Data_P[:,dim-1], 1);
    Data_P = Data_P[::-1,:]; #Invierte el orden de las filas
    plt.xlim([0, C])
    plt.xticks(np.arange(0,C+0.5,1))
    plt.ylim([0, F])
    plt.yticks(np.arange(0,F+0.5,1))
    plt.imshow(Data_P, origin='upper', cmap='jet',alpha=0.4,interpolation='bilinear', vmin=0, vmax=1)
    plt.grid(color = 'gray', linestyle = ':', linewidth = 0.5)
    plt.tight_layout()
     #Guarda la imagen
    if save_graph:
        plt.savefig(FileName)
    #Muestra la imagen
    if show_graph:
        plt.draw()
        plt.pause(0.001)
    plt.clf()
    return path_Opt


#Esto es lo que se debe llamar desde otro archivo,
#enviando los nombres de los csv correspondientes y haciendo
#import ADPDic_Data
#[A,D,P,Net,F,C,Cob] = ADPDic_Data.GenData_ADPDic(DataFile,DataDang,'A.csv','D.csv','P.csv','Dict.json');
#path_Opt = ADPDic_Data.Graph_OptGraph_Opt(A,Net,F,C,DataDang,True,'Graph.png',False,0);

#Carga los archivos producto de la exploración
#DataFile = 'DataPrueba.csv';
#DataDang = 'MatrizPeligro.csv';
#Genera los datos: Matriz de adyacencia, matriz de distancias, matriz de peligrosidad y el diccionario del grafo
#[A,D,P,Net,F,C,Cob] = GenData_ADPDic(DataFile,DataDang,'A.csv','D.csv','P.csv','Dict.json');
#Genera el grafo y determina la ruta óptima
#0: NSGAII; 1:MOEAD4
#path_Opt = Graph_Opt(A,Net,F,C,DataDang,True,'Graph.png',True,0);

#with open('Dic.json') as fid:
#    data = json.load(fid)