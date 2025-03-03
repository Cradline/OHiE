# Inverskinematikk for en 4-DOF RRRR seriemanipulator:
# Gitt (X, Y, Z) koordinater og pitch vinkel 'Alpha'
# Regner ut rotasjonsvinkel for hvert ledd og returnerer vinklene. 


# Importer
import logging
from math import *

# Logging: Feil, Varsler, Info & Debug
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:

    # Servos telles fra bunn til topp.
    # Lenkeparametre:
    l1 = 8.00       # Distansen fra senter av basen (servo1) til servo2: 6.10 cm.
                    # Må kanskje padde l1 litt mtp hjul og chassis
    l2 = 6.50       # Distansen fra servo2 til servo3: 10.16 cm.
    l3 = 6.20       # Distansen fra servo3 til servo4: 9.64 cm.
    l4 = 10.00      # Distansen fra servo4 aksen til tuppen på kloa: 10.0 cm.

    def __init__(self, arm_type):
        # Kan legge til prismatiske ledd her senere 
        self.arm_type = arm_type
        self.arm_type == 'arm'

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # Kan endre lengden på lenker her dersom prismatiske ledd introduseres
        # eller hvis manuell justering av parametre blir nødvendig
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):
        # Henter ut lengdene på lenkene
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}
    

    def getRotationAngle(self, coordinate_data, Alpha):
        # Gitt koordinater og pitch angle 'Alpha', returnerer vinkler for hvert ledd.
        # Returnerer 'False' hvis ingen gyldig løsning.
        # Alpha er vinkelen mellom ende-effektor og XY-planet i grader.
        # Alpha er også vinkelen mellom CD og PC.
        # P(X,Y,Z) er ende-effektoren. O er origo, projisert på bakken under servo1.
        # Punkt A er leddet mellom l1 og l2, punkt B er leddet mellom l2 og l3
        # og punkt C er leddet mellom l3 og l4. CD er ortogonal med PD og Z-aksen.
        # Vinkler er representert på følgende måte: Vinkel mellom AB og BC er ABC.

        X, Y, Z = coordinate_data

        # Theta_6 = rotasjonsvinkel til baseledd (servo1); rotasjon rundt Z_0.
        theta6 = degrees(atan2(Y, X))

        P_O = sqrt(X*X + Y*Y)                   # Distanse fra Origo til ende-effektor
        CD = self.l4 * cos(radians(Alpha))      # Lengde (X) fra ledd-C til ende-effektor (hjelpe-punkt D)
        PD = self.l4 * sin(radians(Alpha))      # Høyde (Z) fra ledd-C til ende-effektor, (hjelpe-punkt D). Alpha pos -> PD = pos, Alpha neg -> PD = neg.
        AF = P_O - CD                           # Distanse fra Origo (eller A) til hjelpe-punkt F, langs X i XZ.  
        CF = Z - self.l1 - PD                   # Høyde (Z) mellom C og hjelpe-punkt F
        AC = sqrt(pow(AF, 2) + pow(CF, 2))      # Distance mellom punkt A og C, via hjelpe-punkt F

        # Sjekker at høydeverdi er konsistent med verdensrammen.
        if round(CF, 4) < -self.l1:
            logger.debug('Høyde under 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # Summen av de to sidene er mindre enn den tredje
            logger.debug('Ugyldig koblingsmekanisme, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False
        
        # Theta_4. phi_B = vinkel ABC, mellom AB og BC
        cos_ABC = round((pow(self.l2, 2) + pow(self.l3, 2) - pow(AC, 2))/(2*self.l2*self.l3), 4) # Law of cosines
        if abs(cos_ABC) > 1:
            logger.debug('Ugyldig koblingsmekanisme, abs(cos_phi_B(%s)) > 1', cos_ABC)
            return False                    # Sjekker for gyldig verdi
        ABC = acos(cos_ABC)             # Finner vinkel phi_B i [rad]
        theta4 = 180.0 - degrees(ABC)     # Konverterer til grader

        # Theta_5. 
        CAF = acos(AF / AC)         # CAF = vinkel mellom AF og CF. 
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) # Law of cosines
        if abs(cos_BAC) > 1:
            logger.debug('Ugyldig koblingsmekanisme, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False            # Sjekker for gyldig verdi
        if CF < 0:                  # Hvis CF er negativ, betyr det at punkt F er over punkt C. zf flag settes til negativ for å justere vinkel CAF
            zf_flag = -1            # Hvis F er under punkt C, er CAF positiv
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        # Theta_3
        theta3 = Alpha - theta5 + theta4


        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # Returns the angles if there is a solution
            
if __name__ == '__main__':
    ik = IK('arm')
    #ik.setLinkLength(L1=ik.l1 + 1.30, L4=ik.l4)
    print('Link length：', ik.getLinkLength())
    #print(ik.getRotationAngle((0, ik.l4, ik.l1 + ik.l2 + ik.l3), 0))