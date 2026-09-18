"""Typeset the book as a PDF.

    .venv/bin/python book/build_pdf.py            # -> docs/the-blind-spot.pdf

Needs reportlab (pip install reportlab). That is a dependency of the BOOK, not
of the study: requirements.txt still lists only numpy, scipy and matplotlib,
and tests.py does not import any of this.

The input is the chapter files in this directory, in the order given by PARTS.
It understands the Markdown subset the chapters are written in: headings,
paragraphs, bullet and numbered lists, pipe tables, fenced code blocks,
blockquotes, and inline bold / italic / code.
"""

import os
import re
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, CondPageBreak, Frame,
                                KeepTogether, NextPageTemplate, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table,
                                TableStyle)
from reportlab.platypus.tableofcontents import TableOfContents

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "the-blind-spot.pdf")

PARTS = [
    ("Part 0", "Foundations", [
        "01-what-this-is-about.md",
        "02-frames-and-transforms.md",
        "03-the-camera.md",
        "04-matrices-and-svd.md",
        "05-feedback-control.md",
    ]),
    ("Part 1", "Visual servoing, derived", [
        "06-the-servo-loop.md",
        "07-interaction-matrix.md",
        "08-control-law-and-motion.md",
        "09-the-harness.md",
    ]),
    ("Part 2", "Four ways to go blind", [
        "10-camera-retreat.md",
        "11-feature-dropout.md",
        "12-two-features.md",
        "13-collapsed-target.md",
    ]),
    ("Part 3", "Fixes, and what they cost", [
        "14-the-partition.md",
        "15-truncation.md",
        "16-the-switch.md",
        "17-the-guard.md",
    ]),
    ("Part 4", "Making sure it is true", [
        "18-verification.md",
        "19-dead-claims.md",
        "20-rendered-camera.md",
    ]),
    ("Part 5", "On a robot", [
        "21-ros2-from-zero.md",
        "22-the-guard-node.md",
        "23-the-gazebo-cell.md",
        "24-kinematics-and-perception.md",
        "25-what-the-arm-found.md",
        "26-the-folding-part.md",
    ]),
    ("Part 6", "Using it", [
        "27-limits.md",
        "28-telling-the-story.md",
        "29-glossary.md",
    ]),
]

# ---------------------------------------------------------------- fonts
DJ = "/usr/share/fonts/truetype/dejavu"
for name, file in (("Body", "DejaVuSans.ttf"), ("Body-Bold", "DejaVuSans-Bold.ttf"),
                   ("Body-Italic", "DejaVuSans-Oblique.ttf"),
                   ("Mono", "DejaVuSansMono.ttf"), ("Mono-Bold", "DejaVuSansMono-Bold.ttf")):
    pdfmetrics.registerFont(TTFont(name, os.path.join(DJ, file)))
pdfmetrics.registerFontFamily("Body", normal="Body", bold="Body-Bold",
                              italic="Body-Italic", boldItalic="Body-Bold")

INK = colors.HexColor("#14171c")
SOFT = colors.HexColor("#5b6472")
RULE = colors.HexColor("#c9cfd8")
CODEBG = colors.HexColor("#f2f4f7")
ACCENT = colors.HexColor("#1d6f5c")

S = {
    "body": ParagraphStyle("body", fontName="Body", fontSize=10, leading=15.2,
                           spaceAfter=7, textColor=INK, alignment=TA_LEFT),
    "h1": ParagraphStyle("h1", fontName="Body-Bold", fontSize=20, leading=25,
                         spaceBefore=0, spaceAfter=12, textColor=INK),
    "h2": ParagraphStyle("h2", fontName="Body-Bold", fontSize=13, leading=17,
                         spaceBefore=14, spaceAfter=5, textColor=INK),
    "h3": ParagraphStyle("h3", fontName="Body-Bold", fontSize=11, leading=15,
                         spaceBefore=10, spaceAfter=3, textColor=ACCENT),
    "bullet": ParagraphStyle("bullet", fontName="Body", fontSize=10, leading=15,
                             leftIndent=14, bulletIndent=3, spaceAfter=3,
                             textColor=INK),
    "quote": ParagraphStyle("quote", fontName="Body-Italic", fontSize=10,
                            leading=15.5, leftIndent=14, rightIndent=8,
                            spaceBefore=5, spaceAfter=8,
                            textColor=colors.HexColor("#2b3a4a"),
                            borderPadding=(0, 0, 0, 6)),
    "code": ParagraphStyle("code", fontName="Mono", fontSize=8.4, leading=11.6,
                           textColor=colors.HexColor("#101418")),
    "cell": ParagraphStyle("cell", fontName="Body", fontSize=8.6, leading=11.6,
                           textColor=INK),
    "cellh": ParagraphStyle("cellh", fontName="Body-Bold", fontSize=8.6,
                            leading=11.6, textColor=INK),
    "title": ParagraphStyle("title", fontName="Body-Bold", fontSize=34,
                            leading=40, alignment=TA_CENTER, textColor=INK),
    "sub": ParagraphStyle("sub", fontName="Body", fontSize=13, leading=19,
                          alignment=TA_CENTER, textColor=SOFT),
    "partno": ParagraphStyle("partno", fontName="Body-Bold", fontSize=13,
                             leading=18, alignment=TA_CENTER, textColor=ACCENT),
    "parttitle": ParagraphStyle("parttitle", fontName="Body-Bold", fontSize=24,
                                leading=30, alignment=TA_CENTER, textColor=INK),
}


# ------------------------------------------------------------- inline markup
def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def inline(t):
    """Markdown inline -> reportlab markup.

    Code spans are lifted out first and put back last, so that bold or italic
    WRAPPING a code span (**the matrix `L`**) still parses, and so that a `*`
    inside a code span is never read as emphasis.
    """
    spans = []

    def stash(m):
        spans.append(m.group(1))
        return "\x00%d\x00" % (len(spans) - 1)

    s = re.sub(r"`([^`]+)`", stash, t)
    s = esc(s)
    s = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", s)
    s = re.sub(r"(?<![\*\w])\*([^*\n]+)\*(?!\*)", r"<i>\1</i>", s)
    s = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", s)            # links -> text
    for i, code in enumerate(spans):
        s = s.replace("\x00%d\x00" % i,
                      '<font face="Mono" size="9" backColor="#eef1f5">%s</font>'
                      % esc(code))
    return s


# ------------------------------------------------------------- block parsing
def table_flowable(rows, width):
    head, body = rows[0], rows[1:]
    data = [[Paragraph(inline(c), S["cellh"]) for c in head]]
    data += [[Paragraph(inline(c), S["cell"]) for c in r] for r in body]
    ncol = max(len(r) for r in data)
    data = [r + [Paragraph("", S["cell"])] * (ncol - len(r)) for r in data]
    first = min(0.42, 1.0 / ncol + 0.10) if ncol > 2 else 0.5
    rest = (1.0 - first) / (ncol - 1) if ncol > 1 else 1.0
    widths = [width * first] + [width * rest] * (ncol - 1)
    t = Table(data, colWidths=widths, hAlign="LEFT", repeatRows=1)
    t.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("LINEBELOW", (0, 0), (-1, 0), 0.9, RULE),
        ("LINEBELOW", (0, 1), (-1, -2), 0.3, colors.HexColor("#e6e9ee")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#f6f8fa")),
    ]))
    return t


def code_flowable(lines, width):
    body = "<br/>".join(esc(l).replace(" ", "&nbsp;") or "&nbsp;" for l in lines)
    p = Paragraph(body, S["code"])
    t = Table([[p]], colWidths=[width], hAlign="LEFT")
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), CODEBG),
        ("BOX", (0, 0), (-1, -1), 0.4, RULE),
        ("LEFTPADDING", (0, 0), (-1, -1), 8),
        ("RIGHTPADDING", (0, 0), (-1, -1), 8),
        ("TOPPADDING", (0, 0), (-1, -1), 7),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
    ]))
    return t


def parse(md, width, chapter_no):
    """Markdown subset -> list of flowables."""
    flow, lines, i = [], md.splitlines(), 0
    para, bullets, rows = [], [], []

    def flush_para():
        if para:
            flow.append(Paragraph(inline(" ".join(para)), S["body"]))
            para.clear()

    def flush_bullets():
        if bullets:
            for mark, text in bullets:
                flow.append(Paragraph(inline(text), S["bullet"],
                                      bulletText=mark))
            flow.append(Spacer(1, 4))
            bullets.clear()

    def flush_rows():
        if rows:
            flow.append(Spacer(1, 3))
            flow.append(table_flowable(rows, width))
            flow.append(Spacer(1, 9))
            rows.clear()

    def flush_all():
        flush_para(); flush_bullets(); flush_rows()

    while i < len(lines):
        ln = lines[i]
        if ln.startswith("```"):
            flush_all()
            i += 1
            buf = []
            while i < len(lines) and not lines[i].startswith("```"):
                buf.append(lines[i]); i += 1
            i += 1
            flow.append(code_flowable(buf, width))
            flow.append(Spacer(1, 8))
            continue
        if ln.startswith("|") and ln.rstrip().endswith("|"):
            cells = [c.strip() for c in ln.strip().strip("|").split("|")]
            if not re.match(r"^[\s:\-|]+$", ln):          # skip the --- row
                rows.append(cells)
            i += 1
            continue
        flush_rows() if rows and not ln.startswith("|") else None
        if not ln.strip():
            flush_para(); flush_bullets()
            i += 1
            continue
        if ln.startswith("# "):
            flush_all()
            title = ln[2:].strip()
            flow.append(Paragraph(inline(title), S["h1"]))
            flow[-1].toc_entry = (0, title, chapter_no)
            i += 1
            continue
        if ln.startswith("## "):
            flush_all()
            flow.append(CondPageBreak(34 * mm))
            flow.append(Paragraph(inline(ln[3:].strip()), S["h2"]))
            i += 1
            continue
        if ln.startswith("### "):
            flush_all()
            flow.append(CondPageBreak(28 * mm))
            flow.append(Paragraph(inline(ln[4:].strip()), S["h3"]))
            i += 1
            continue
        if ln.startswith("> "):
            flush_all()
            buf = []
            while i < len(lines) and lines[i].startswith("> "):
                buf.append(lines[i][2:]); i += 1
            flow.append(Paragraph(inline(" ".join(buf)), S["quote"]))
            continue
        m = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)$", ln)
        if m:
            flush_para()
            indent, mark, text = m.group(1), m.group(2), m.group(3)
            j = i + 1
            while (j < len(lines) and lines[j].startswith("  ")
                   and not re.match(r"^\s*([-*]|\d+\.)\s+", lines[j])):
                text += " " + lines[j].strip(); j += 1
            i = j
            bullets.append(("\u2022" if mark in "-*" else mark, text))
            continue
        para.append(ln.strip())
        i += 1
    flush_all()
    return flow


# --------------------------------------------------------------- the document
class Book(BaseDocTemplate):
    def __init__(self, path, **kw):
        BaseDocTemplate.__init__(self, path, pagesize=A4,
                                 leftMargin=24 * mm, rightMargin=22 * mm,
                                 topMargin=20 * mm, bottomMargin=18 * mm, **kw)
        frame = Frame(self.leftMargin, self.bottomMargin, self.width,
                      self.height, id="body")
        self.addPageTemplates([
            PageTemplate(id="plain", frames=[frame]),
            PageTemplate(id="chapter", frames=[frame], onPage=self.decorate),
        ])
        self.chapter = ""
        self.part = ""

    def handle_documentBegin(self):
        # multiBuild runs the whole story twice to resolve the table of
        # contents; without this the second pass starts with the running head
        # left over from the end of the first.
        BaseDocTemplate.handle_documentBegin(self)
        self.chapter = ""
        self.part = ""

    def decorate(self, canv, doc):
        canv.saveState()
        canv.setFont("Body", 7.5)
        canv.setFillColor(SOFT)
        canv.drawString(self.leftMargin, A4[1] - 13 * mm, self.part)
        canv.drawRightString(A4[0] - self.rightMargin, A4[1] - 13 * mm,
                             "The Blind Spot")
        canv.setStrokeColor(RULE)
        canv.setLineWidth(0.4)
        canv.line(self.leftMargin, A4[1] - 15 * mm,
                  A4[0] - self.rightMargin, A4[1] - 15 * mm)
        canv.drawCentredString(A4[0] / 2, 11 * mm, str(canv.getPageNumber()))
        canv.restoreState()

    def afterFlowable(self, flowable):
        part = getattr(flowable, "part_name", None)
        if part:
            self.part = part
        entry = getattr(flowable, "toc_entry", None)
        if entry:
            level, title, number = entry
            self.chapter = title
            self.notify("TOCEntry", (level, title, self.page))


def build():
    width = A4[0] - 46 * mm
    doc = Book(OUT, title="The Blind Spot",
               author="Saurabh Khimesra", subject="Visual servoing failure modes")
    story = [Spacer(1, 52 * mm),
             Paragraph("The Blind Spot", S["title"]),
             Spacer(1, 7 * mm),
             Paragraph("How a vision-guided robot goes blind, "
                       "and one line of code that notices", S["sub"]),
             Spacer(1, 26 * mm),
             Paragraph("A complete account of the project: the algorithms from "
                       "first principles, every number that decided something, "
                       "the ROS 2 stack, and the mistakes, in order.", S["sub"]),
             Spacer(1, 40 * mm),
             Paragraph("github.com/SaurabhKhimesra/blind-spot", S["sub"]),
             PageBreak()]

    toc = TableOfContents()
    toc.levelStyles = [ParagraphStyle("toc", fontName="Body", fontSize=10,
                                      leading=17, textColor=INK)]
    story += [Paragraph("Contents", S["h1"]), Spacer(1, 4 * mm), toc,
              PageBreak()]

    story += [Paragraph("How to read this", S["h1"])]
    for line in [
        "Every chapter opens <b>in plain words</b>, with no jargon and no "
        "symbols, then gives the precise version with the measured numbers, "
        "and closes with <b>Say it like this</b> - a spoken version you could "
        "use in an interview.",
        "The book assumes nothing. Part 0 builds frames, cameras, matrices "
        "and feedback from scratch; every term is defined where it first "
        "appears, and chapter 29 is a glossary plus every number in the book "
        "with what it means.",
        "<b>Never seen robotics:</b> chapters 1 to 5 in order, then 10, 14, "
        "17, then 26.",
        "<b>Know control, new to visual servoing:</b> 6 to 9, then Part 2, "
        "then 17.",
        "<b>Preparing for an interview tomorrow:</b> 28, then 19, then 27, "
        "then 23.",
        "<b>Reviewing the engineering:</b> 9, 18, 19, 27.",
        "Every number quoted here is produced by a script in the repository "
        "and, where it decided something, locked by a regression test. Where "
        "a claim of mine turned out to be wrong, the retraction is in the "
        "book next to the claim rather than deleted - chapter 19 is nothing "
        "else.",
    ]:
        story.append(Paragraph(line, S["body"]))

    n = 0
    for i, (part_no, part_title, files) in enumerate(PARTS):
        head = Paragraph(part_title, S["parttitle"])
        head.part_name = "%s \u00b7 %s" % (part_no, part_title)
        story += [NextPageTemplate("plain"), PageBreak(),
                  Spacer(1, 60 * mm),
                  Paragraph(part_no, S["partno"]), head,
                  NextPageTemplate("chapter"), PageBreak()]
        for f in files:
            n += 1
            path = os.path.join(HERE, f)
            if not os.path.exists(path):
                print("MISSING", f)
                continue
            story += parse(open(path).read(), width, n)
            story.append(PageBreak())
    doc.multiBuild(story)
    print("wrote", OUT, "(%d chapters)" % n)


if __name__ == "__main__":
    sys.exit(build())
