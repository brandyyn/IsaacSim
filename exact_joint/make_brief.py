"""Create the two-page presentation brief from verified local simulation captures."""

from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4,landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph
from reportlab.lib.utils import ImageReader

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"output/pdf/Exact_Knee_Cable_FEM_Brief.pdf"
OUT.parent.mkdir(parents=True,exist_ok=True)
W,H=landscape(A4)
NAVY=colors.HexColor("#123040")
TEAL=colors.HexColor("#006F79")
GRAY=colors.HexColor("#506572")
PALE=colors.HexColor("#EDF4F6")
RED=colors.HexColor("#9D352B")


def paragraph(c,text,x,top,width,size=11,color=GRAY,leading=None):
    style=ParagraphStyle("body",fontName="Helvetica",fontSize=size,leading=leading or size*1.35,textColor=color)
    p=Paragraph(text,style)
    _,height=p.wrap(width,1000)
    p.drawOn(c,x,top-height)
    return height


def footer(c,page):
    c.setStrokeColor(colors.HexColor("#D6E1E6"))
    c.line(36,26,W-36,26)
    paragraph(c,"EXACT SOURCE JOINT  |  RESEARCH PROTOTYPE  |  06 SEP 2026",36,20,650,7.5)
    paragraph(c,str(page)+" / 2",W-72,20,40,7.5)


def crop_image(c,path,x,y,width,height):
    reader=ImageReader(str(path))
    iw,ih=reader.getSize()
    scale=max(width/iw,height/ih)
    dw,dh=iw*scale,ih*scale
    c.saveState()
    clip=c.beginPath()
    clip.rect(x,y,width,height)
    c.clipPath(clip,stroke=0,fill=0)
    c.drawImage(reader,x+(width-dw)/2,y+(height-dh)/2,dw,dh,mask="auto")
    c.restoreState()


c=canvas.Canvas(str(OUT),pagesize=(W,H))
c.setTitle("Exact knee joint - cable-driven live FEM prototype")
c.setAuthor("Isaac Sim joint research project")
paragraph(c,"DESIGN CHECKPOINT",36,H-30,650,10,TEAL)
paragraph(c,"Your original joint. At the knee only.",36,H-49,W-72,27,NAVY)
paragraph(c,"Original JSON geometry, rigid end plates and cable-load-driven finite-element response inside Isaac Sim.",36,H-87,W-72,11)
crop_image(c,ROOT/"exact_joint/neutral_preview.png",36,195,510,287)
crop_image(c,ROOT/"exact_joint/whole_leg_preview.png",579,195,225,287)
paragraph(c,"Original fold pattern with red cross-plate cables and black top routing reference.",36,184,510,9)
paragraph(c,"One origami knee; rigid thigh, shank and foot.",579,184,225,9)
for x,title,text in (
    (36,"EXACT GEOMETRY","28 shared vertices<br/>50 panels / 76 edges<br/>30 x 30 x 26.4 mm neutral surface"),
    (302,"CONFIRMED STACK","PLA: 0.4 mm<br/>PET film: 80 micrometres<br/>0.2 mm exposed PET gap is provisional"),
    (568,"CABLE INPUTS","Compression, two-plane bending, twist<br/>Experimental nonnegative tensions in N<br/>Both square plates remain rigid")):
    paragraph(c,title,x,149,240,10,TEAL)
    paragraph(c,text,x,129,240,10.5)
c.setFillColor(colors.HexColor("#FFF0EB"))
c.roundRect(36,39,W-72,48,6,stroke=0,fill=1)
paragraph(c,"EXPLORATORY - NOT A LARGE-FOLDING OR STRENGTH VALIDATION",48,78,W-96,10,RED)
paragraph(c,"The current live FEM is quasistatic and small-strain. Mesh convergence fails; no material allowables, fatigue life or motor rating are claimed.",48,62,W-96,9.5,RED)
footer(c,1)
c.showPage()

paragraph(c,"LIVE FEM WORKSHOP",36,H-30,650,10,TEAL)
paragraph(c,"How to demonstrate it and what to report",36,H-49,W-72,25,NAVY)
paragraph(c,"Use the \"Exact knee - cable FEM\" panel. The custom solver is controlled here, not by the Isaac timeline Play button.",36,H-87,W-72,10.5)
steps=[
    ("01","Inspect","Choose Whole leg or Exact knee close-up. The displayed movement is true scale, with no hidden amplification."),
    ("02","Apply cable load","Select a pattern, enter tension in N per active cable and click Apply. Start at 0.1 N; 0.25 N is the demo input, not a hardware recommendation."),
    ("03","See FEM results","Stress / material toggles the color field. Read bend, twist, compression, PET/PLA stress and strain in the panel."),
    ("04","Change one parameter","Adjust PET gap, PET/PLA modulus or mesh refinement. Rebuild at zero load, then repeat the same cable load."),
    ("05","Save and compare","Save FEM results and inputs creates a new timestamped record. Pause freezes the solver; Neutral removes cable loads.")]
top=H-120
for number,title,text in steps:
    paragraph(c,number,36,top,30,13,TEAL)
    paragraph(c,"<b>"+title+"</b> - "+text,77,top,422,10)
    top-=58

paragraph(c,"CHECKS AND LIMITS",535,H-120,268,11,TEAL)
paragraph(c,"<b>7 / 7</b> numeric tests passed.<br/><b>7 / 7</b> cable patterns exercised on the single knee.<br/>No hip or ankle origami copies in the active scene.<br/>Rigid roof distances preserved to display precision.",535,H-143,268,10)
paragraph(c,"MESH SENSITIVITY AT 0.1 N / CABLE",535,H-225,268,9,TEAL)
rows=[("Refinement","Compression","CW twist"),("1","2.521 um","-0.003275 deg"),("2","4.984 um","-0.005009 deg"),("3","6.207 um","-0.006089 deg")]
y=H-250
for index,row in enumerate(rows):
    c.setFillColor(PALE if index%2==0 else colors.white)
    c.rect(530,y-22,274,25,stroke=0,fill=1)
    for x,text in zip((535,617,702),row):
        paragraph(c,text,x,y,95,8.5,NAVY if index==0 else GRAY)
    y-=25
paragraph(c,"Refinement 2 to 3 changes compression by 24.6% and twist by 21.6%. Both exceed the 5% convergence criterion.",535,y-10,268,10,RED)

c.setFillColor(PALE)
c.roundRect(36,108,465,67,6,stroke=0,fill=1)
paragraph(c,"WHAT IS BEING SOLVED",48,165,440,9,TEAL)
paragraph(c,"Quadratic TET10 PLA/PET elements, four Gauss points, perfect bonds. Interior FEM coordinates are condensed exactly; cable forces determine the rigid lower plate pose. Displacement, strain and stress are recovered live.",48,148,440,9.5)
paragraph(c,"Next: converge and calibrate the creases, then validate nonlinear folding, guide clearance and loaded-leg behavior. No ML policy has been trained.",535,145,268,9.5)
paragraph(c,"Reproduction and full assumptions: exact_joint/README.md. Source JSON SHA256 begins d578618a0f90. Starting shared source checkpoint: 65e98b1c.",36,88,W-72,8.5)
paragraph(c,"Element reference: FEBio Theory Manual 4.7, section 4.1.4. Model-limit reference: SOLIDWORKS Help, Large Displacement Solution.",36,65,W-72,8.2)
c.linkURL("https://help.febio.org/docs/FEBioTheory-4-7/TM47-Subsection-4.1.4.html",(36,51,425,68),relative=0)
c.linkURL("https://help.solidworks.com/2022/English/SolidWorks/cworks/c_Large_Displacement_Solution.htm",(425,51,W-36,68),relative=0)
footer(c,2)
c.save()
print(OUT)
