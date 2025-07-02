import os
import csv
import math
from statistics import mean, stdev, variance

# Function to convert strings to float if possible
def convert_to_float(row):
    return [float(item) if item.replace('.', '', 1).isdigit() else item for item in row]

raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"
principal_path = os.path.join(raiz_path,"Principal.csv")
resumen_path = os.path.join(raiz_path,"Resumen.csv")


with open(principal_path, 'w', newline='', encoding='utf-8') as principalfile:
    with open(resumen_path, 'w', newline='', encoding='utf-8') as resumenfile:    
        writer_principal = csv.writer(principalfile)
        writer_resumen = csv.writer(resumenfile)
        
        writer_principal.writerow(["Robots", "Duracion (min)", "Prueba", "Percentil (%)", "Distancia", "Peligrosidad", "Cobertura", "%Cobertura"])
        writer_resumen.writerow(["Robots", "Duracion (min)", "Percentil (%)", "Metrica", "Promedio", "DesvEst", "Varianza"])
        
        lista_metricas = ["Distancia", "Peligrosidad", "Cobertura", "%Cobertura"]
        lista_per = [25,50,75,100]
        
        for cantidadRobots in range(1, 4):
            for tiempoPrueba in [5, 10, 15]:
                
                # Preparo las listas para promedio, desvEst y varianza
                # Esta matriz consta de los cuatro percentiles por distancia, peligrosidad, cobertura y %cobertura
                datos = [[[],[],[],[]], [[],[],[],[]], [[],[],[],[]], [[],[],[],[]]]
                
                for prueba in range(1, 14):
                    prueba_path = raiz_path+str(cantidadRobots)+"_robot/"+str(tiempoPrueba)+"-"+str(prueba)
                    Cob_path = os.path.join(prueba_path, "path_objective_Cob.csv")
                    
                    with open(Cob_path, 'r') as cobfile:
                        reader = [convert_to_float(row) for row in csv.reader(cobfile)]  # Read all rows into a list
                    
                    # Calculate the total number of rows
                    total_rows = len(reader)
                    
                    # Identify the row indices for 25%, 50%, 75%, and 100%
                    row_25 = math.ceil(0.25 * total_rows) - 1  # Adjust for 0-based indexing
                    row_50 = math.ceil(0.50 * total_rows) - 1
                    row_75 = math.ceil(0.75 * total_rows) - 1
                    row_100 = total_rows - 1  # Last row
                    
                    # Extract the last three columns from each identified row
                    columns_25 = reader[row_25][-3:]
                    columns_50 = reader[row_50][-3:]
                    columns_75 = reader[row_75][-3:]
                    columns_100 = reader[row_100][-3:]
                    
                    percentiles = [columns_25, columns_50, columns_75, columns_100]
                    
                    # Calculo el porcentaje de cobertura y lo agrego como cuarta columna
                    for per in range(4):
                        percentiles[per].append(percentiles[per][2]/(24*24)*100)  
                        for i in range(4):
                            datos[i][per].append(percentiles[per][i])
                    
                        writer_principal.writerow([cantidadRobots,tiempoPrueba,prueba,lista_per[per],percentiles[per][0],percentiles[per][1],percentiles[per][2],percentiles[per][3]])  
                
                estadisticas = [[[],[],[],[]],[[],[],[],[]],[[],[],[],[]],[[],[],[],[]]]
                
                for dato in range(4):
                    for per in range(4):
                        estadisticas[dato][per].append(mean(datos[dato][per]))
                        estadisticas[dato][per].append(stdev(datos[dato][per]))
                        estadisticas[dato][per].append(variance(datos[dato][per]))
                    
                        writer_resumen.writerow( [cantidadRobots, tiempoPrueba, lista_per[dato], lista_metricas[per], mean(datos[per][dato]), stdev(datos[per][dato]), variance(datos[per][dato])])
       
print("\n¡Terminado!")