import os
import ast
import re

import platform

blackHole = ">/dev/null 2>&1"
if "Windows" == platform.system():
    blackHole = " > NUL"

absPath = os.path.abspath(__file__)
if os.path.islink(absPath):
    absPath = os.readlink(os.path.abspath(__file__))
basePath = os.path.join(os.path.dirname(absPath), "../../")

dflOWLFilename = os.path.join(basePath, "owl/SOMA_MERGED.owl")
dflQueryOWLFilename = os.path.join(basePath, "owl/SOMA_DFL_query.owl")
dflResponseFilename = os.path.join(basePath, "owl/SOMA_DFL_response.owl")
owlFolder = os.path.join(basePath, "owl")
resourcesFolder = os.path.join(basePath, "resources")
dflUseMatchFilename = os.path.join(resourcesFolder, "DFLUseMatch.res")
koncludeBinary = os.path.join(basePath, "bin/Konclude")
if "Windows" == platform.system():
    koncludeBinary = os.path.join(basePath, "bin/Konclude.exe")

prefixes = [
    ('', 'http://www.ease-crc.org/ont/DLQuery.owl#'), 
    ('owl', 'http://www.w3.org/2002/07/owl#'),
    ('rdf', 'http://www.w3.org/1999/02/22-rdf-syntax-ns#'),
    ('xml', 'http://www.w3.org/XML/1998/namespace'),
    ('xsd', 'http://www.w3.org/2001/XMLSchema#'),
    ('rdfs', 'http://www.w3.org/2000/01/rdf-schema#'),
    ('dul', 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#'),
    ('soma', 'http://www.ease-crc.org/ont/SOMA.owl#'),
    ('dfl', 'http://www.ease-crc.org/ont/SOMA_DFL.owl#')]

prefixesDFL = [
    ('', 'http://www.ease-crc.org/ont/SOMA_DFL.owl#'),
    ('dfl', 'http://www.ease-crc.org/ont/SOMA_DFL.owl#'),
    ('owl', 'http://www.w3.org/2002/07/owl#'),
    ('rdf', 'http://www.w3.org/1999/02/22-rdf-syntax-ns#'),
    ('xml', 'http://www.w3.org/XML/1998/namespace'),
    ('xsd', 'http://www.w3.org/2001/XMLSchema#'),
    ('rdfs', 'http://www.w3.org/2000/01/rdf-schema#'),
    ('dul', 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#'),
    ('soma', 'http://www.ease-crc.org/ont/SOMA.owl#'),
    ('DUL', 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#'),
    ('SOMA', 'http://www.ease-crc.org/ont/SOMA.owl#'),
    ('usd', 'https://ease-crc.org/ont/USD.owl#'),
    ('USD', 'https://ease-crc.org/ont/USD.owl#')]

def expandName(conceptName, prefs=None):
    if None == prefs:
        prefs = prefixes
    if ('<' == conceptName[0]) or (':' not in conceptName):
        return conceptName
    prefix = conceptName[:conceptName.find(':')]
    suffix = conceptName[conceptName.find(':') + 1:]
    retq = suffix
    for p, exp in prefs:
        if prefix == p:
            retq = "<" + exp + retq + ">"
    return retq

def contractName(conceptName, prefs=None):
    if None == prefs:
        prefs = prefixes
    if ('<' == conceptName[0]):
        conceptName = conceptName[1:]
    if ('>' == conceptName[-1]):
        conceptName = conceptName[:-1]
    retq = conceptName
    for p, exp in prefs:
        if conceptName.startswith(exp):
            retq = p + ":" + retq[len(exp):]
    return retq        

def queryHeader():
    retq = ""
    for p, exp in prefixes:
        retq = retq + ("Prefix(%s:=<%s>)\n" % (p, exp))
    retq = retq + ("Ontology(<http://www.ease-crc.org/ont/DLQuery.owl>\n")
    retq = retq + ("Import(<file://./SOMA_DFL.owl>)\n\n")
    return retq

def inferTransitiveClosure(c, closedGraph, graph, ignoreConcepts=set([])):
    todo = set(graph[c])
    closedGraph[c] = set([])
    while todo:
        sc = todo.pop()
        if (sc not in closedGraph[c]) and (sc not in ignoreConcepts):
            closedGraph[c].add(sc)
            if sc in graph:
                todo = todo.union(graph[sc])
    return closedGraph

def flipGraph(graph):
    retq = {}
    for v in graph:
        for u in graph[v]:
            if u not in retq:
                retq[u] = set([])
            retq[u].add(v)
    return retq

def _parseHomebrew(filename):
    inEq = False
    inSubcl = False
    superclasses = {}
    aux = []
    iriPLen = len("        <Class IRI=\"")
    with open(filename) as file_in:
        for l in file_in:
            if inEq:
                if "    </EquivalentClasses>\n" == l:
                    inEq = False
                    for c in aux:
                        if c not in superclasses:
                            superclasses[c] = set([])
                        for d in aux:
                            if d != c:
                                superclasses[c].add(d)
                    aux = []
                else:
                    if l.startswith("        <Class IRI=\""):
                        aux.append(l[iriPLen:-4])
            elif inSubcl:
                if "    </SubClassOf>\n" == l:
                    inSubcl = False
                    if 2 <= len(aux):
                        if aux[0] not in superclasses:
                            superclasses[aux[0]] = set([])
                        superclasses[aux[0]].add(aux[1])
                    aux = []
                else:
                    if l.startswith("        <Class IRI=\""):
                        aux.append(l[iriPLen:-4])
            else:
                if "    <SubClassOf>\n" == l:
                    inSubcl = True
                elif "    <EquivalentClasses>\n" == l:
                    inEq = True
    return superclasses

def parseResponse(filename):
    return _parseHomebrew(filename)

def runQuery(query):
    query = queryHeader() + query + "\n)\n"
    with open(dflQueryOWLFilename, "w") as outfile:
        outfile.write(query)
    os.system("cd %s && %s classification -i %s -o %s %s" % (owlFolder, koncludeBinary, dflQueryOWLFilename, dflResponseFilename, blackHole))
    return parseResponse(dflResponseFilename)

__dispositionSubsumptionCache__ = None
__dispositionSubsumptionCacheFlipped__ = None
__useMatchCache__ = None

def __loadUseMatchCache():
    global __useMatchCache__
    __useMatchCache__ = [tuple([':'+y for y in ast.literal_eval(x)]) for x in open(dflUseMatchFilename).read().splitlines() if x.strip()]

def __getQueryName(conceptName):
    shortName = contractName(conceptName)
    return shortName[shortName.find(':'):] + ".QUERY"

def __isQueryConcept(conceptName):
    return conceptName.startswith('http://www.ease-crc.org/ont/DLQuery.owl#') and conceptName.endswith('.QUERY')

def __filterApproximates(conceptName):
    if conceptName.startswith("dfl:Approximate"):
        return None
    return conceptName

def buildCache():
    global __dispositionSubsumptionCache__
    global __dispositionSubsumptionCacheFlipped__
    superclasses = runQuery("")
    subclasses = flipGraph(superclasses)
    dispositionConcept = expandName("soma:Disposition")[1:-1]
    dispositionTClosure = inferTransitiveClosure(dispositionConcept, {}, subclasses)[dispositionConcept]
    query = ""
    for c in dispositionTClosure:
        query = query + ("EquivalentClasses(%s ObjectSomeValuesFrom(dul:hasQuality %s))\n" % (__getQueryName(c), contractName(c)))
    __dispositionSubsumptionCache__ = runQuery(query)
    __dispositionSubsumptionCacheFlipped__ = flipGraph(__dispositionSubsumptionCache__)

def whatsImpossible(usecache=True):
    retq = set()
    nothing = expandName("owl:Nothing")[1:-1] # Trim <>, Konclude's XML does not include these
    if usecache and __dispositionSubsumptionCache__:
        superclasses = __dispositionSubsumptionCache__
    else:
        superclasses = runQuery("")
    return sorted([y for y in [contractName(x) for x in superclasses.keys() if nothing in superclasses[x]] if __filterApproximates(y)])
    
## Loosely speaking: what is this?
def whatSuperclasses(concept, usecache=True):
    concept = expandName(concept)
    concept = concept[1:-1] # Trim <>, Konclude's XML does not include these
    inferredSuperclasses = set([])
    if usecache and __dispositionSubsumptionCache__ and (concept in __dispositionSubsumptionCache__):
        inferredSuperclasses = inferTransitiveClosure(concept, {}, __dispositionSubsumptionCache__)[concept]
    elif (not usecache) or (not __dispositionSubsumptionCache__):
        superclasses = runQuery("")
        if concept in superclasses:
            inferredSuperclasses = inferTransitiveClosure(concept, {}, superclasses)[concept]
    return sorted([y for y in list(set([contractName(x) for x in inferredSuperclasses if (not __isQueryConcept(x))])) if __filterApproximates(y)])

## Loosely speaking: what kinds of this are there?
def whatSubclasses(concept, usecache=True):
    concept = expandName(concept)
    concept = concept[1:-1] # Trim <>, Konclude's XML does not include these
    inferredSubclasses = set([])
    if usecache and __dispositionSubsumptionCacheFlipped__ and (concept in __dispositionSubsumptionCacheFlipped__):
        inferredSubclasses = inferTransitiveClosure(concept, {}, __dispositionSubsumptionCacheFlipped__)[concept]
    elif (not usecache) or (not __dispositionSubsumptionCacheFlipped__):
        subclasses = flipGraph(runQuery(""))
        if concept in subclasses:
            inferredSubclasses = inferTransitiveClosure(concept, {}, subclasses)[concept]
    return sorted([y for y in list(set([contractName(x) for x in inferredSubclasses if (not __isQueryConcept(x))])) if __filterApproximates(y)])

## Loosely speaking: what can you use to do this action to this particular object?
def whatToolsCanPerformTaskOnObject(conceptTask, conceptPatient, usecache=True):
    if not __useMatchCache__:
        __loadUseMatchCache()
    retq = set([])
    inferredSubclassesTask = set([])
    inferredSuperclassesPatient = set([])
    conceptTask = expandName(conceptTask)[1:-1] # Trim <>, Konclude's XML does not include these
    conceptPatient = expandName(conceptPatient)[1:-1]
    if usecache and __dispositionSubsumptionCacheFlipped__ and (conceptTask in __dispositionSubsumptionCacheFlipped__) and __dispositionSubsumptionCache__ and (conceptPatient in __dispositionSubsumptionCache__):
        inferredSubclassesTask = inferTransitiveClosure(conceptTask, {}, __dispositionSubsumptionCacheFlipped__)[conceptTask]
        inferredSuperclassesPatient = inferTransitiveClosure(conceptPatient, {}, __dispositionSubsumptionCache__)[conceptPatient]
    elif (not usecache) or (not __dispositionSubsumptionCacheFlipped__) or (not __dispositionSubsumptionCache__):
        superclasses = runQuery("")
        subclasses = flipGraph(superclasses)
        if conceptTask in subclasses:
            inferredSubclassesTask = inferTransitiveClosure(conceptTask, {}, subclasses)[conceptTask]
        if conceptPatient in superclasses:
            inferredSuperclassesPatient = inferTransitiveClosure(conceptPatient, {}, superclasses)[conceptPatient]
    inferredSubclassesTask.add(conceptTask)
    inferredSuperclassesPatient.add(conceptPatient)
    for t in __useMatchCache__:
        if expandName(t[0], prefs=prefixesDFL)[1:-1] in inferredSubclassesTask:
            if expandName(t[2], prefs=prefixesDFL)[1:-1] in inferredSuperclassesPatient:
                conceptInstrument = expandName(t[1], prefs=prefixesDFL)[1:-1]
                retq = retq.union(inferTransitiveClosure(conceptInstrument, {}, __dispositionSubsumptionCacheFlipped__)[conceptInstrument])
                retq.add(conceptInstrument)
    return sorted([y for y in [contractName(x) for x in retq] if __filterApproximates(y)])

print("Caching disposition queries ...")
buildCache()
print("    Done.")

def parseClassAssertions(filename):
    inCA = False
    cas = []
    aux = [None, None]
    iriPLen = len("        <Class IRI=\"")
    iriNIPLen = len("        <NamedIndividual IRI=\"")
    with open(filename) as file_in:
        for l in file_in:
            if inCA:
                if "    </ClassAssertion>\n" == l:
                    inCA = False
                    cas.append(aux)
                    aux = [None, None]
                else:
                    if l.startswith("        <Class IRI=\""):
                        aux[1] = "<"+(l[iriPLen:-4])+">"
                    elif l.startswith("        <NamedIndividual IRI=\""):
                        aux[0] = "<"+(l[iriNIPLen:-4])+">"
            else:
                if "    <ClassAssertion>\n" == l:
                    inCA = True
    return cas

def findClassAssertionsFor(name):
    name = expandName(name, prefs=prefixesDFL)
    cas = parseClassAssertions(dflResponseFilename)
    return set([x[1] for x in cas if name == x[0]])

def findClassAssertionsOfTypes(types):
    types = set([expandName(x, prefs=prefixesDFL) for x in types])
    cas = parseClassAssertions(dflResponseFilename)
    return set([x[0] for x in cas if x[1] in types])

def findObjectsOfTypes(types, ontologyFilename):
    os.system("cd %s && %s realization -i %s -o %s %s" % (owlFolder, koncludeBinary, ontologyFilename, dflResponseFilename, blackHole))
    return findClassAssertionsOfTypes(types)

def whatIsThisObject(name, ontologyFilename):
    os.system("cd %s && %s realization -i %s -o %s %s" % (owlFolder, koncludeBinary, ontologyFilename, dflResponseFilename, blackHole))
    return findClassAssertionsFor(expandName(name, prefs=prefixesDFL))

def whereToStoreObject(name, ontologyFilename):
    classes = whatIsThisObject(name, ontologyFilename)
    rawAllClasses = set()
    for c in classes:
        rawAllClasses = rawAllClasses.union(whatSuperclasses(c))
    dflClasses = [x for x in rawAllClasses if x.lower().startswith("dfl:")]
    specifics = []
    for c in dflClasses:
        if 1 == len(classes.intersection([expandName(x, prefs=prefixesDFL) for x in whatSubclasses(c)])):
            specifics.append(c)
    stores = set()
    for s in dflClasses:#specifics:
        stores = stores.union([expandName(x, prefs=prefixesDFL) for x in whatToolsCanPerformTaskOnObject("dfl:store.v.wn.possession..place", s)])
    return findObjectsOfTypes(stores, ontologyFilename)

def isItAnOpenFlap(name, ontologyFilename):
    classes = whatIsThisObject(name, ontologyFilename)
    return "<https://ease-crc.org/ont/usd/box_TBox.owl#OpenedFlap>" in classes

def isItAClosedFlap(name, ontologyFilename):
    classes = whatIsThisObject(name, ontologyFilename)
    return "<https://ease-crc.org/ont/usd/box_TBox.owl#ClosedFlap>" in classes

