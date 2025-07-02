library(ggplot2)
library(RColorBrewer)
library(pheatmap)
library(dplyr)
library(tidyr)
library(GGally)
library(ggpubr)
library(hrbrthemes)
library(viridis)
library(plot3D)
library(MASS)
library(ggtern)
library(scales)

# Mark the current directory
#setwd('C:\\Users\\ccalderon\\OneDrive - Estudiantes ITCR\\PROE\\BancoExperimentos\\Sala_Investigacion\\Pruebas de optimización\\GraficasFinales')


#Archivo general completo, por percentiles
df_Gen <- read.csv("Summary_DPCb.csv", header = TRUE, sep = ",") #\t
df_Gen[["Dist"]] <- df_Gen[["Dist"]] * 0.8  #convierte la distancia en cm

#Archivo de coberturas con 3 robots durante 15 min, por instante del tiempo
dfCob15 <- read.csv("Cob_15min_3Rob.csv", header = TRUE, sep = ",")

#Frame generales para 5min, 10min y 15min, versión horizontal
df5 <- data.frame(subset(df_Gen,TotalTime==5 & Prueba==1,select=c(Rob,Time)))
rownames(df5) <- NULL
for (i in 1:13) {
  df5 <- cbind(df5,  subset(df_Gen,TotalTime==5 & Prueba==i,select=c(Dist,Pel,Cob)))
  D <- paste0("Dist_", i)  # Construye el nombre de la columna
  P <- paste0("Pel_", i)
  C <- paste0("Cob_", i)
  names(df5)[names(df5) == "Dist"] <- D #Cambia el nombre a la columna
  names(df5)[names(df5) == "Pel"] <- P
  names(df5)[names(df5) == "Cob"] <- C
  rownames(df5) <- NULL
}
#Estadísticos de la distancia por percentil
MeanDist <- rowMeans(df5[, c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_10","Dist_11","Dist_12","Dist_13")])
df5 <- cbind(df5, MeanDist)
STDDist <- apply(subset(df5, select = c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_10","Dist_11","Dist_12","Dist_13")),1, sd)
df5 <- cbind(df5, STDDist)
#Estadísticos de la peligrosidad por percentil
MeanPel <- rowMeans(df5[, c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_10","Pel_11","Pel_12","Pel_13")])
df5 <- cbind(df5, MeanPel)
STDPel <- apply(subset(df5, select = c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_10","Pel_11","Pel_12","Pel_13")),1, sd)
df5 <- cbind(df5, STDPel)
#Estadísticos de la cobertura por percentil
MeanCob <- rowMeans(df5[, c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_10","Cob_11","Cob_12","Cob_13")])
df5 <- cbind(df5, MeanCob)
STDCob <- apply(subset(df5, select = c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_10","Cob_11","Cob_12","Cob_13")),1, sd)
df5 <- cbind(df5, STDCob)

df10 <- data.frame(subset(df_Gen,TotalTime==10 & Prueba==1,select=c(Rob,Time)))
rownames(df10) <- NULL
for (i in 1:13) {
  if (i != 10) {
  df10 <- cbind(df10,  subset(df_Gen,TotalTime==10 & Prueba==i,select=c(Dist,Pel,Cob)))
  D <- paste0("Dist_", i)  # Construye el nombre de la columna
  P <- paste0("Pel_", i)
  C <- paste0("Cob_", i)
  names(df10)[names(df10) == "Dist"] <- D #Cambia el nombre a la columna
  names(df10)[names(df10) == "Pel"] <- P
  names(df10)[names(df10) == "Cob"] <- C
  rownames(df10) <- NULL
  }
}
#Estadísticos de la distancia por percentil 
MeanDist <- rowMeans(df10[, c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_11","Dist_12","Dist_13")])
df10 <- cbind(df10, MeanDist)
STDDist <- apply(subset(df10, select = c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_11","Dist_12","Dist_13")),1, sd)
df10 <- cbind(df10, STDDist)
#Estadísticos de la peligrosidad por percentil 
MeanPel <- rowMeans(df10[, c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_11","Pel_12","Pel_13")])
df10 <- cbind(df10, MeanPel)
STDPel <- apply(subset(df10, select = c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_11","Pel_12","Pel_13")),1, sd)
df10 <- cbind(df10, STDPel)
#Estadísticos de la cobertura por percentil 
MeanCob <- rowMeans(df10[, c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_11","Cob_12","Cob_13")])
df10 <- cbind(df10, MeanCob)
STDCob <- apply(subset(df10, select = c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_11","Cob_12","Cob_13")),1, sd)
df10 <- cbind(df10, STDCob)

df15 <- data.frame(subset(df_Gen,TotalTime==15 & Prueba==1,select=c(Rob,Time)))
rownames(df15) <- NULL
for (i in 1:13) {
  df15 <- cbind(df15,  subset(df_Gen,TotalTime==15 & Prueba==i,select=c(Dist,Pel,Cob)))
  D <- paste0("Dist_", i)  # Construye el nombre de la columna
  P <- paste0("Pel_", i)
  C <- paste0("Cob_", i)
  names(df15)[names(df15) == "Dist"] <- D #Cambia el nombre a la columna
  names(df15)[names(df15) == "Pel"] <- P
  names(df15)[names(df15) == "Cob"] <- C
  rownames(df15) <- NULL
}
#Estadísticos de la distancia por percentil
MeanDist <- rowMeans(df15[, c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_10","Dist_11","Dist_12","Dist_13")])
df15 <- cbind(df15, MeanDist)
STDDist <- apply(subset(df15, select = c("Dist_1","Dist_2","Dist_3","Dist_4","Dist_5","Dist_6","Dist_7","Dist_8","Dist_9","Dist_10","Dist_11","Dist_12","Dist_13")),1, sd)
df15 <- cbind(df15, STDDist)
#Estadísticos de la peligrosidad por percentil 
MeanPel <- rowMeans(df15[, c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_10","Pel_11","Pel_12","Pel_13")])
df15 <- cbind(df15, MeanPel)
STDPel <- apply(subset(df15, select = c("Pel_1","Pel_2","Pel_3","Pel_4","Pel_5","Pel_6","Pel_7","Pel_8","Pel_9","Pel_10","Pel_11","Pel_12","Pel_13")),1, sd)
df15 <- cbind(df15, STDPel)
#Estadísticos de la cobertura por percentil 
MeanCob <- rowMeans(df15[, c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_10","Cob_11","Cob_12","Cob_13")])
df15 <- cbind(df15, MeanCob)
STDCob <- apply(subset(df15, select = c("Cob_1","Cob_2","Cob_3","Cob_4","Cob_5","Cob_6","Cob_7","Cob_8","Cob_9","Cob_10","Cob_11","Cob_12","Cob_13")),1, sd)
df15 <- cbind(df15, STDCob)

# df5 <- read.csv("Cob_5min_Gen.csv", header = TRUE, sep = ";") #\t
# for (i in 1:13) {
#   col_name <- paste0("Dist_", i)  # Construye el nombre de la columna
#   df5[[col_name]] <- df5[[col_name]] * 0.8  #convierte la distancia en cm
# }
# 
# df10 <- read.csv("Cob_10min_Gen.csv", header = TRUE, sep = ";")
# for (i in 1:13) {
#   col_name <- paste0("Dist_", i)  # Construye el nombre de la columna
#   df10[[col_name]] <- df10[[col_name]] * 0.8  #convierte la distancia en cm
# }
# 
# df15 <- read.csv("Cob_15min_Gen.csv", header = TRUE, sep = ";")
# for (i in 1:13) {
#   col_name <- paste0("Dist_", i)  # Construye el nombre de la columna
#   df15[[col_name]] <- df15[[col_name]] * 0.8  #convierte la distancia en cm
# }

#Filtra la Distancia, peligrosidad y cobertura alcanzada con 1, 2, 3 robots a los 5, 10 y 15 min
df_End <- subset(df_Gen, (TotalTime == 5 & Time == 5) | (TotalTime == 10 & Time == 10) | (TotalTime == 15 & Time == 15), select = c(Rob,Time,Dist,Pel,Cob))
dfDPC5 <- subset(df_Gen, TotalTime == 5 & Time == 5, select = c(Rob,Dist,Pel,Cob))
dfDPC10 <- subset(df_Gen, TotalTime == 10 & Time == 10, select = c(Rob,Dist,Pel,Cob))  
dfDPC15 <- subset(df_Gen, TotalTime == 15 & Time == 15, select = c(Rob,Dist,Pel,Cob))

#Filtra los resultados con más del 80% de cobertura en las corridas de 15min y 3 robots
Lim_Cob = 80.00
dfPF <- subset(df_Gen, Rob == 3 & TotalTime == 15.00 & Time == 15.00 & Cob >= Lim_Cob, select = c(Dist,Pel,Cob))

#Genera los estadísticos finales de todas las corridas
STD_df5_End <- subset(df_Gen, Time == 5) %>%
  group_by(Rob) %>%
  summarise(
    MeanDist = mean(Dist),
    StdDist = sd(Dist),
    MeanPel = mean(Pel),
    StdPel = sd(Pel),
    MeanCob = mean(Cob),
    StdCob = sd(Cob))

STD_df10_End <- subset(df_Gen, Time == 10) %>%
  group_by(Rob) %>%
  summarise(
    MeanDist = mean(Dist),
    StdDist = sd(Dist),
    MeanPel = mean(Pel),
    StdPel = sd(Pel),
    MeanCob = mean(Cob),
    StdCob = sd(Cob))

STD_df15_End <- subset(df_Gen, Time == 15) %>%
  group_by(Rob) %>%
  summarise(
    MeanDist = mean(Dist),
    StdDist = sd(Dist),
    MeanPel = mean(Pel),
    StdPel = sd(Pel),
    MeanCob = mean(Cob),
    StdCob = sd(Cob))

#Carga las celdas visitadas en cada corrida de 15min con 3 robots
Color15_1 <- as.matrix(read.csv("Color_15_1.csv", header = FALSE, sep = ","))
#dfColor15_1 <- read.csv("Color_15_1.csv", header = FALSE, sep = ",")
Color15_2 <- as.matrix(read.csv("Color_15_2.csv", header = FALSE, sep = ","))
Color15_3 <- as.matrix(read.csv("Color_15_3.csv", header = FALSE, sep = ","))
Color15_4 <- as.matrix(read.csv("Color_15_4.csv", header = FALSE, sep = ","))
Color15_5 <- as.matrix(read.csv("Color_15_5.csv", header = FALSE, sep = ","))
Color15_6 <- as.matrix(read.csv("Color_15_6.csv", header = FALSE, sep = ","))
Color15_7 <- as.matrix(read.csv("Color_15_7.csv", header = FALSE, sep = ","))
Color15_8 <- as.matrix(read.csv("Color_15_8.csv", header = FALSE, sep = ","))
Color15_9 <- as.matrix(read.csv("Color_15_9.csv", header = FALSE, sep = ","))
Color15_10 <- as.matrix(read.csv("Color_15_10.csv", header = FALSE, sep = ","))
Color15_11 <- as.matrix(read.csv("Color_15_11.csv", header = FALSE, sep = ","))
Color15_12 <- as.matrix(read.csv("Color_15_12.csv", header = FALSE, sep = ","))
Color15_13 <- as.matrix(read.csv("Color_15_13.csv", header = FALSE, sep = ","))

#Gráficos de promedios y desviación estándar, en los percentiles de cada tiempo de ejecución
ggplot(data=df5, aes(x=Time, y=MeanCob, group=Rob)) +
  geom_point(aes(x=Time, y=Cob_1, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_2, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_3, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_4, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_5, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_6, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_7, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_8, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_9, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_10, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_11, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_12, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_13, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_line(aes(color=as.factor(Rob)),size=1,linetype="solid") + geom_point(aes(color=as.factor(Rob)),size=3) +
  geom_errorbar(data=df5, aes(ymin=MeanCob-STDCob, ymax=MeanCob+STDCob,color=as.factor(Rob)), width=0.5,size=1,linetype="solid") +
  scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 1.25)) +
  scale_y_continuous(limits=c(0,100),breaks = seq(0, 100, by = 10)) + #xlim(c(0,16)) + ylim(c(0,110)) +
  labs(color="Robots",x = "Percentile threshold of execution time (min)", y = "Mean coverage percentage (%)") +
  scale_color_brewer(palette="Dark2") + #scale_color_manual(values=c("#999999", "#E69F00", "#56B4E9"))
  theme_bw() + theme(legend.position = c(0.5,0.91),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))

ggplot(data=df10, aes(x=Time, y=MeanCob, group=Rob)) +
  geom_point(aes(x=Time, y=Cob_1, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_2, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_3, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_4, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_5, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_6, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_7, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_8, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_9, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  #geom_point(aes(x=Time, y=Cob_10, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_11, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_12, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_13, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_line(aes(color=as.factor(Rob)),size=1,linetype="solid") + geom_point(aes(color=as.factor(Rob)),size=3) +
  geom_errorbar(data=df10, aes(ymin=MeanCob-STDCob, ymax=MeanCob+STDCob,color=as.factor(Rob)), width=0.5,size=0.5,linetype="solid") +
  scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 1.25)) +
  scale_y_continuous(limits=c(0,100),breaks = seq(0, 100, by = 10)) +
  labs(color="Robots",x = "Percentile threshold of execution time (min)", y = "Mean coverage percentage (%)") +
  scale_color_brewer(palette="Dark2") +
  theme_bw() + theme(legend.position = c(0.5,0.91),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))

ggplot(data=df15, aes(x=Time, y=MeanCob, group=Rob)) +
  geom_point(aes(x=Time, y=Cob_1, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_2, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_3, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_4, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_5, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_6, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_7, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_8, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_9, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_10, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_11, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_12, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_point(aes(x=Time, y=Cob_13, group=Rob,color=as.factor(Rob)),size=4,alpha=0.15) +
  geom_line(aes(color=as.factor(Rob)),size=1,linetype="solid") + geom_point(aes(color=as.factor(Rob)),size=3) +
  geom_errorbar(data=df15, aes(ymin=MeanCob-STDCob, ymax=MeanCob+STDCob,color=as.factor(Rob)), width=0.5,size=0.5,linetype="solid") +
  scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 1.25)) +
  scale_y_continuous(limits=c(0,100),breaks = seq(0, 100, by = 10)) +
  labs(color="Robots",x = "Percentile threshold of execution time (min)", y = "Mean coverage percentage (%)") +
  scale_color_brewer(palette="Dark2") +
  theme_bw() + theme(legend.position = c(0.5,0.91),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))#theme(legend.position = "none")

#Gráficas de cobertura y ajuste: 15 min con 3 robots
ggplot(data=dfCob15, aes(x=Time, y=Cob_Porc,group=Prueb)) + 
  geom_point(aes(color=as.factor(Prueb)),size=0.75) + 
  geom_smooth(aes(color=as.factor(Prueb)),size=0.5,alpha=0.5,level=0.99) +
  scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 2.5)) +
  scale_y_continuous(limits=c(-10,100),breaks = seq(0, 100, by = 10)) +
  labs(color="Experiment",x = "Execution time (min)", y = "Coverage percentage (%)") + 
  #scale_color_brewer(palette="Dark2") +
  theme_bw() + theme(legend.position = "none")

ggplot(data=dfCob15, aes(x=Time, y=Cob_Porc)) + 
  geom_point(aes(color=as.factor(Prueb)),size=1) + 
  geom_smooth(size=1,color="red",alpha=0.8,level=0.99) + #method="loess",span=0.5
  scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 2.5)) +
  scale_y_continuous(limits=c(-10,100),breaks = seq(0, 100, by = 10)) +
  labs(x = "Execution time (min)", y = "Coverage percentage (%)") + 
  #scale_color_brewer(palette="Dark2") +
  theme_bw() + theme(legend.position = "none")

#Mapa de calor de la cobertura
ColorTotal <- Color15_1+Color15_2+Color15_3+Color15_4+Color15_5+Color15_6+Color15_7+Color15_8+Color15_9+Color15_10+Color15_11+Color15_12+Color15_13
#ColorTotal <- ColorTotal[1:24,2:25]
ColorTotal <- ColorTotal[rev(1:nrow(ColorTotal)), ] #Invierte el orden de las filas

dfColor <- as.data.frame(ColorTotal)
dfColor$row <- 1:nrow(dfColor)  # Añadir una columna para las filas
df_long <- reshape2::melt(dfColor, id.vars = "row")  # Convertir a formato largo

ggplot(data=df_long,aes(x = variable, y = row)) +
  #geom_tile(aes(x=variable,y=row,fill=value)) + coord_fixed() +
  geom_raster(aes(fill = value)) + coord_fixed() +
  #scale_fill_gradientn(colours = terrain.colors(-7)) +
  #scale_fill_viridis_c(option = "mako") +
  #scale_colour_brewer(palette = "Set1")+
  #scale_fill_distiller(palette = "Spectral",type = "div")+
  scale_fill_gradient(low = "black", high = "white",na.value = NA)+
  theme_void() + theme(legend.position = "none")

#Gráfico multivariables con ejes paralelos
colnames(dfDPC15) <- c("Rob","Accumulated \n path distance (cm)","Accumulated \n path danger","Coverage (%)")
dfDPC15$Rob <- as.factor(dfDPC15$Rob)
ggparcoord(data = dfDPC15, columns = 2:4, groupColumn = 1, alphaLines = 0.7, showPoints = TRUE,splineFactor = 10, scale = "uniminmax",mapping = ggplot2::aes(linewidth = 0.75)) +
  #geom_line(size=2) +
  labs(color="Robots",x = " ", y = " ") +
  #scale_color_brewer(palette="Set1") +
  scale_colour_manual(values = c("1" = "darkgray", "2" = "dodgerblue", "3" = "midnightblue"))+ #https://derekogle.com/NCGraphing/resources/colors
  theme_minimal()
#otra forma
parcoord(dfDPC15[,2:4], col = dfDPC15[,1],var.label = TRUE)
# 
# #Pareto Front con los resultados con al menos Lim_Cob de cobertura
# ggplot() + 
#   geom_point(data=dfPF[dfPF$Cob < 90, ], aes(x=Dist, y=Pel),size=4) +
#   geom_text(data=dfPF[dfPF$Cob < 90, ], aes(label=sprintf("%.2f %%", Cob), y = Pel + 0.5), cex=3.5, col="black")+
#   geom_segment(data=dfPF[dfPF$Cob < 90, ],aes(x = Dist, xend = Dist, y = Pel, yend = Pel+2), linetype = "dashed", color = "black")+
#   geom_segment(data=dfPF[dfPF$Cob < 90, ],aes(x = Dist, xend = Dist+50, y = Pel, yend = Pel), linetype = "dashed", color = "black")+
#   #geom_vline(data = dfPF[dfPF$Cob < 90, ], aes(xintercept = Dist), linetype = "dotted", color = "black") +
#   #geom_hline(data = dfPF[dfPF$Cob < 90, ], aes(yintercept = Pel), linetype = "dotted", color = "black")+
#   geom_point(data = dfPF[dfPF$Cob >= 90 & dfPF$Cob < 95, ], aes(x = Dist, y = Pel), shape = 18, size = 4.5, color = "blue") +
#   geom_text(data=dfPF[dfPF$Cob >= 90 & dfPF$Cob < 95, ], aes(label=sprintf("%.2f %%", Cob), y = Pel + 0.5), cex=3.5, col="blue")+
#   geom_segment(data=dfPF[dfPF$Cob >= 90 & dfPF$Cob < 95, ],aes(x = Dist, xend = Dist, y = Pel, yend = Pel+2), linetype = "dashed", color = "blue")+
#   geom_segment(data=dfPF[dfPF$Cob >= 90 & dfPF$Cob < 95, ],aes(x = Dist, xend = Dist+50, y = Pel, yend = Pel), linetype = "dashed", color = "blue")+
#   geom_point(data = dfPF[dfPF$Cob >= 95, ], aes(x = Dist, y = Pel), shape = 17, size = 4, color = "red") +
#   geom_text(data=dfPF[dfPF$Cob >= 95, ], aes(label=sprintf("%.2f %%", Cob), y = Pel - 0.5), cex=3.5, col="red")+
#   geom_segment(data=dfPF[dfPF$Cob >= 95, ],aes(x = Dist, xend = Dist, y = Pel, yend = Pel+2), linetype = "dashed", color = "red")+
#   geom_segment(data=dfPF[dfPF$Cob >= 95, ],aes(x = Dist, xend = Dist+50, y = Pel, yend = Pel), linetype = "dashed", color = "red")+
#   labs(x = "Accumulated path distance (cm)", y = "Accumulated path danger") + 
#   theme_minimal() + theme(legend.position = "none")
# 
# #Pareto front (Dist,Pel) en Box Plot o Error bar
# ggplot(data=STD_df5_End, aes(x=MeanDist, y=MeanPel, group=Rob)) +
#   geom_point(data=subset(df_Gen,TotalTime==5&Time==5),aes(x=Dist, y=Pel, group=Rob,color=as.factor(Rob)),size=4,alpha=0.25) +
#   geom_errorbar(data=STD_df5_End, aes(ymin=MeanPel-StdPel, ymax=MeanPel+StdPel,color=as.factor(Rob)),size=1,linetype="solid") +
#   geom_errorbarh(data=STD_df5_End, aes(xmin=MeanDist-StdDist, xmax=MeanDist+StdDist,color=as.factor(Rob)),size=1,linetype="solid") +
#   #scale_x_continuous(limits=c(0,15.5),breaks=seq(0, 15.5, by = 1.25)) +
#   #scale_y_continuous(limits=c(0,100),breaks = seq(0, 100, by = 10)) + #xlim(c(0,16)) + ylim(c(0,110)) +
#   labs(color="Robots",x = "Accumulated path distance (cm)", y = "Accumulated path danger") +
#   scale_color_brewer(palette="Dark2") + scale_color_manual(values=c("#999999", "#E69F00", "#56B4E9"))+
#   theme_bw() + theme(legend.position = c(0.5,0.91),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))

#Gráfico de barras: agrupado por tiempo de ejecución y cantidad de robots
#Cobertura en cada percentil del tiempo de ejecución
MeanDif <- numeric(15)
StdDif <- numeric(15)
for (i in 1:3){
  k = 5*(i-1)+1
  MeanDif[k] <- df5$MeanCob[k]
  MeanDif[k+1] <- df5$MeanCob[k+1]-df5$MeanCob[k]
  MeanDif[k+2] <- df5$MeanCob[k+2]-df5$MeanCob[k+1]
  MeanDif[k+3] <- df5$MeanCob[k+3]-df5$MeanCob[k+2]
  MeanDif[k+4] <- df5$MeanCob[k+4]-df5$MeanCob[k+3]
  StdDif[k] <- df5$STDCob[k]
  StdDif[k+1] <- df5$STDCob[k+1]-df5$STDCob[k]
  StdDif[k+2] <- df5$STDCob[k+2]-df5$STDCob[k+1]
  StdDif[k+3] <- df5$STDCob[k+3]-df5$STDCob[k+2]
  StdDif[k+4] <- df5$STDCob[k+4]-df5$STDCob[k+3]
}
df5_PercDif = data.frame(Rob <- df5$Rob,
                         Time <- df5$Time,
                         Mean <- MeanDif,
                         Std <- StdDif)
colnames(df5_PercDif) <- c("Rob", "Time" ,"Mean","Std")
ggplot() + 
  geom_bar(data = df5_PercDif, aes(fill=as.factor(Time), y=Mean, x=Rob), position=position_stack(reverse = TRUE), stat="identity",width=0.95) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_1),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_2),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_3),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_4),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_5),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_6),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_7),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_8),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_9),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_10),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_11),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_12),size=3,alpha=0.25) +
  geom_point(data=subset(df5,Time==5),aes(x=Rob, y=Cob_13),size=3,alpha=0.25) +
  geom_errorbar(data=subset(df5,Time==5), aes(x = Rob, ymin=MeanCob-STDCob, ymax=MeanCob+STDCob),size=1,linetype="solid",width=.2,position=position_dodge(.9)) +
  geom_text(data = df5, aes(x = Rob, y=MeanCob, label=paste0(round(MeanCob,2),"%")), vjust=-0.3,hjust=-0.3, color="black", size=3.5)+
  labs(fill="Minutes",x = " ", y = "Average coverage percentage (%)") +
  scale_fill_brewer(palette="Greens") +
  scale_y_continuous(limits=c(0,110),breaks = seq(0, 100, by = 10))+
  theme_bw() + theme(legend.position = c(0.5,0.95),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))+
  facet_wrap(~"Execution over 5 minutes")

MeanDif10 <- numeric(15)
StdDif10 <- numeric(15)
for (i in 1:3){
  k = 5*(i-1)+1
  MeanDif10[k] <- df10$MeanCob[k]
  MeanDif10[k+1] <- df10$MeanCob[k+1]-df10$MeanCob[k]
  MeanDif10[k+2] <- df10$MeanCob[k+2]-df10$MeanCob[k+1]
  MeanDif10[k+3] <- df10$MeanCob[k+3]-df10$MeanCob[k+2]
  MeanDif10[k+4] <- df10$MeanCob[k+4]-df10$MeanCob[k+3]
  StdDif10[k] <- df10$STDCob[k]
  StdDif10[k+1] <- df10$STDCob[k+1]-df10$STDCob[k]
  StdDif10[k+2] <- df10$STDCob[k+2]-df10$STDCob[k+1]
  StdDif10[k+3] <- df10$STDCob[k+3]-df10$STDCob[k+2]
  StdDif10[k+4] <- df10$STDCob[k+4]-df10$STDCob[k+3]
}
df10_PercDif = data.frame(Rob <- df10$Rob,
                         Time <- df10$Time,
                         Mean <- MeanDif10,
                         Std <- StdDif10)
colnames(df10_PercDif) <- c("Rob", "Time" , "Mean","Std")
ggplot() + 
  geom_bar(data = df10_PercDif, aes(fill=as.factor(Time), y=Mean, x=Rob),position=position_stack(reverse = TRUE), stat="identity",width=0.95) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_1),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_2),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_3),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_4),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_5),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_6),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_7),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_8),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_9),size=3,alpha=0.25) +
  #geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_10),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_11),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_12),size=3,alpha=0.25) +
  geom_point(data=subset(df10,Time==10),aes(x=Rob, y=Cob_13),size=3,alpha=0.25) +
  geom_errorbar(data=subset(df10,Time==10), aes(x = Rob, ymin=MeanCob-STDCob, ymax=MeanCob+STDCob),size=1,linetype="solid",width=.2,position=position_dodge(.9)) +
  geom_text(data = df10, aes(x = Rob, y=MeanCob, label=paste0(round(MeanCob,2),"%")), vjust=-0.3,hjust=-0.3, color="black", size=3.5)+
  labs(fill="Minutes",x = "Number of robots", y = " ") +
  scale_fill_brewer(palette="Oranges") +
  scale_y_continuous(limits=c(0,110),breaks = seq(0, 100, by = 10))+
  #scale_fill_viridis(discrete = T, option = "G") +
  theme_bw() + theme(legend.position = c(0.5,0.95),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))+
  facet_wrap(~"Execution over 10 minutes")

MeanDif15 <- numeric(15)
StdDif15 <- numeric(15)
for (i in 1:3){
  k = 5*(i-1)+1
  MeanDif15[k] <- df15$MeanCob[k]
  MeanDif15[k+1] <- df15$MeanCob[k+1]-df15$MeanCob[k]
  MeanDif15[k+2] <- df15$MeanCob[k+2]-df15$MeanCob[k+1]
  MeanDif15[k+3] <- df15$MeanCob[k+3]-df15$MeanCob[k+2]
  MeanDif15[k+4] <- df15$MeanCob[k+4]-df15$MeanCob[k+3]
  StdDif15[k] <- df15$STDCob[k]
  StdDif15[k+1] <- df15$STDCob[k+1]-df15$STDCob[k]
  StdDif15[k+2] <- df15$STDCob[k+2]-df15$STDCob[k+1]
  StdDif15[k+3] <- df15$STDCob[k+3]-df15$STDCob[k+2]
  StdDif15[k+4] <- df15$STDCob[k+4]-df15$STDCob[k+3]
}
df15_PercDif = data.frame(Rob <- df15$Rob,
                         Time <- df15$Time,
                         Mean <- MeanDif15,
                         Std <- StdDif15)
colnames(df15_PercDif) <- c("Rob", "Time" , "Mean","Std")
ggplot() + 
  geom_bar(data=df15_PercDif, aes(fill=as.factor(Time), y=Mean, x=Rob),position=position_stack(reverse = TRUE), stat="identity",width=0.95) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_1),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_2),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_3),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_4),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_5),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_6),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_7),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_8),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_9),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_10),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_11),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_12),size=3,alpha=0.25) +
  geom_point(data=subset(df15,Time==15),aes(x=Rob, y=Cob_13),size=3,alpha=0.25) +
  geom_errorbar(data=subset(df15,Time==15), aes(x=Rob, ymin=MeanCob-STDCob, ymax=MeanCob+STDCob),size=1,linetype="solid",width=.2,position=position_dodge(.9)) +
  geom_text(data=df15, aes(x = Rob, y=MeanCob, label=paste0(round(MeanCob,2),"%")), vjust=-0.3,hjust=-0.3, color="black", size=3.5)+
  labs(fill="Minutes",x = " ", y = " ") +
  scale_fill_brewer(palette="Purples") +
  scale_y_continuous(limits=c(0,110),breaks = seq(0, 100, by = 10))+
  theme_bw() + theme(legend.position = c(0.5,0.95),legend.direction = "horizontal", legend.box.background = element_rect(color="black", linewidth=0.5))+
  facet_wrap(~"Execution over 15 minutes")

#Juan Ca
#Solo 3 robots y 15 minutos
ggplot(data= dfPF ,aes(y = Pel, x = Dist) ) +
  geom_text(aes(label=sprintf("%.1f %%", Cob), y = Pel - 0.5), cex=3.5, col="black")+
  geom_segment(aes(x = Dist, xend = Dist, y = Pel, yend = Pel+2), linetype = "dashed", color = "black")+
  geom_segment(aes(x = Dist, xend = Dist+50, y = Pel, yend = Pel), linetype = "dashed", color = "black")+
  geom_point(aes(colour= Cob), size = 5, alpha = 1) +
  scale_colour_distiller(palette = "Purples", direction = 1)+ #"BrBG" - "YlOrBr"
  labs(colour = "Coverage (%)", x = "Accumulated path distance (cm)", y= "Accumulated path danger", shape= "Time (min)")+
  theme_bw()+ theme(legend.title =  element_text(size = 9), legend.position = "right" )
#https://ggplot2-book.org/scales-colour.html

####### TERNARIo #####
#Cargar solo los de 15 min
df_End_sub <- subset(df_End, Cob>0 & Time == 15)#Quita los ceros iniciales
#Robots-Time to discrete categories (since it already has discrete values)
df_End_sub$Rob_dis <- factor(df_End_sub$Rob, levels = c(1, 2, 3))
#df_End_sub$Time_discrete <- factor(df_End_sub$Time, levels = c(5, 10, 15))

#Normalizar datos porque tienen proporciones muy distintas
df_End_sub$Dist_norm <- df_End_sub$Dist/norm(as.matrix(df_End_sub$Dist),type="2")#((df_End_sub$Dist - mean(df_End_sub$Dist))/sd(df_End_sub$Dist))#(df_End_sub$Dist - min(df_End_sub$Dist)) / (max(df_End_sub$Dist)-min(df_End_sub$Dist))#
df_End_sub$Dist_norm = df_End_sub$Dist_norm - min(df_End_sub$Dist_norm)
df_End_sub$Cob_norm <- df_End_sub$Cob/norm(as.matrix(df_End_sub$Cob),type="2")#((df_End_sub$Cob - mean(df_End_sub$Cob))/sd(df_End_sub$Cob))#(df_End_sub$Cob - min(df_End_sub$Cob)) / (max(df_End_sub$Cob)-min(df_End_sub$Cob))#
df_End_sub$Cob_norm = df_End_sub$Cob_norm - min(df_End_sub$Cob_norm)
df_End_sub$Pel_norm <- df_End_sub$Pel/norm(as.matrix(df_End_sub$Pel),type="2")#((df_End_sub$Pel - mean(df_End_sub$Pel))/sd(df_End_sub$Pel))#(df_End_sub$Pel - min(df_End_sub$Pel)) / (max(df_End_sub$Pel)-min(df_End_sub$Pel))#
df_End_sub$Pel_norm = df_End_sub$Pel_norm - min(df_End_sub$Pel_norm)

#Normalizar datos con Z score
#df_End_sub$Dist_z <- scale(df_End_sub$Dist)
#df_End_sub$Cob_z <- scale(df_End_sub$Cob)
#df_End_sub$Pel_z <- scale(df_End_sub$Pel)
#df_End_sub$Time_z <- scale(df_End_sub$Time)

#Ahora preparar los datos, para que cada trio represente proporciones de 1
#df_End_sub$Total <- df_End_sub$Dist_z + df_End_sub$Cob_z + df_End_sub$Pel_z
#df_End_sub$Dist_tern <- df_End_sub$Dist_z / df_End_sub$Total
#df_End_sub$Cob_tern <- df_End_sub$Cob_z / df_End_sub$Total
#df_End_sub$Pel_tern <- df_End_sub$Pel_z / df_End_sub$Total

ggtern(data=df_End_sub, aes(x=Dist_norm, y=Cob_norm, z=Pel_norm)) + 
  geom_point(aes(size=Rob_dis, colour=Rob_dis), alpha=0.3)+
  #scale_colour_distiller(palette = "Blues")+
  scale_colour_manual(values = c("1" = "gray50", "2" = "dodgerblue", "3" = "midnightblue"))+
  theme_light() + theme_showarrows() +
  labs(
    x = "Accum.\n distance",
    y = "Coverage",
    z = "Accum.\n danger",
    size= "Robots",
    colour="Time (min)"
  )+
  scale_size_manual(values = c(2, 4, 6)) +
  
  guides(
    size = guide_legend("Robots", override.aes = list(colour = c("gray50", "dodgerblue", "midnightblue"))),
    colour = guide_legend("Robots", override.aes = list(size = c(2, 4, 6)))
  ) +
  #scale_size_continuous(labels = function(x) x / 2) +
  theme(axis.title = element_text(size = 9), legend.title =  element_text(size = 9), legend.position = c(0.9, 0.82) )


