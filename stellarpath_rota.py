# =============================================================================
# STELLARPATH AI — Módulo de Otimização de Rota Lunar
# =============================================================================
# Disciplina : Dynamic Programming
# Projeto    : Global Solution 2026 — Space Connect (FIAP)
# Integrantes: Guilherme de Paula Kuskowski  RM562471
#              Guilherme Eduardo de Lima      RM566045
#              Enzo de Faria Ferreira         RM562448
#
# Estruturas de dados implementadas:
#   - Busca Binária — localização de waypoints por ID
#   - FIFO (fila de waypoints para execução sequencial)
#   - LIFO (pilha de backtracking em rotas bloqueadas)
#   - Fila de prioridade de missões por urgência
#   - Classes FilaFIFO, PilhaLIFO e FilaPrioridade
#     com métodos enfileirar, desenfileirar, peek, is_empty e mostrar
# =============================================================================

import heapq
import math


# =============================================================================
# CONSTANTES DE RISCO
# =============================================================================

RISCO_CRITICO   = 10
RISCO_ALTO      = 7
RISCO_MEDIO     = 4
RISCO_BAIXO     = 1
PENALIDADE_GD   = 1.5
LIMIAR_BLOQUEIO = 10


# =============================================================================
# Implementação de FilaFIFO, PilhaLIFO e FilaPrioridade
# Métodos disponíveis: enfileirar, desenfileirar, peek, is_empty, mostrar
# =============================================================================

class FilaFIFO:
    """
    Fila FIFO (First In, First Out).
    O primeiro waypoint a entrar na fila de navegação
    é o primeiro a ser processado pelo agente robótico.

    Uso no projeto: armazena a sequência de waypoints da
    rota calculada para execução ordenada pelo robô lunar.
    """

    def __init__(self):
        """Construtor: cria a fila de waypoints vazia."""
        self.itens = []

    def enfileirar(self, item):
        """Adiciona um waypoint ao final da fila (FIFO)."""
        self.itens.append(item)
        print(f"  [FILA FIFO] Waypoint '{item}' adicionado à fila de rota.")

    def desenfileirar(self):
        """
        Remove e retorna o primeiro waypoint da fila.
        Equivalente a fila.pop(0).
        """
        if not self.is_empty():
            return self.itens.pop(0)
        return "Fila vazia"

    def peek(self):
        """Mostra o próximo waypoint sem removê-lo."""
        if not self.is_empty():
            return self.itens[0]
        return "Fila vazia"

    def is_empty(self):
        """Verifica se a fila está vazia."""
        return len(self.itens) == 0

    def mostrar(self):
        """Exibe todos os waypoints na fila com print."""
        print(f"  [FILA FIFO] Conteúdo atual ({len(self.itens)} itens):")
        if self.itens:
            for i, item in enumerate(self.itens):
                print(f"    [{i}] {item}")
        else:
            print("    (fila vazia)")
        return self.itens

    def tamanho(self):
        """Retorna quantos waypoints estão na fila."""
        return len(self.itens)


class PilhaLIFO:
    """
    Pilha LIFO (Last In, First Out).
    O último waypoint empilhado é o primeiro a ser processado.

    Uso no projeto: backtracking de rotas bloqueadas.
    Quando um caminho é interditado, o agente desempilha
    waypoints visitados para tentar um percurso alternativo.
    """

    def __init__(self):
        """Construtor: cria a pilha de histórico vazia."""
        self.itens = []

    def empilhar(self, item):
        """Empilha um waypoint no topo (LIFO)."""
        self.itens.append(item)

    def desempilhar(self):
        """
        Remove e retorna o waypoint do topo.
        Usa pop() sem índice — padrão LIFO.
        """
        if not self.is_empty():
            return self.itens.pop()
        return "Pilha vazia"

    def peek(self):
        """Mostra o waypoint no topo sem remover."""
        if not self.is_empty():
            return self.itens[-1]
        return "Pilha vazia"

    def is_empty(self):
        """Verifica se a pilha está vazia."""
        return len(self.itens) == 0

    def mostrar(self):
        """Exibe todos os waypoints empilhados com print (topo ao fundo)."""
        print(f"  [PILHA LIFO] Conteúdo atual ({len(self.itens)} itens) — topo à esquerda:")
        if self.itens:
            for i, item in enumerate(reversed(self.itens)):
                topo = " ← TOPO" if i == 0 else ""
                print(f"    [{i}] {item}{topo}")
        else:
            print("    (pilha vazia)")
        return self.itens

    def tamanho(self):
        """Retorna quantos waypoints estão na pilha."""
        return len(self.itens)


class FilaPrioridade:
    """
    Fila de Prioridade com Heap Mínimo.
    Usada pelo algoritmo de Dijkstra para sempre processar
    o waypoint de menor custo acumulado primeiro.

    Baseada no módulo heapq do Python.
    """

    def __init__(self):
        """Construtor: cria o heap vazio."""
        self.heap = []

    def inserir(self, custo, waypoint_id):
        """
        Insere um waypoint com seu custo na fila de prioridade.
        heappush mantém automaticamente a propriedade do heap.
        """
        heapq.heappush(self.heap, (custo, waypoint_id))

    def remover_minimo(self):
        """
        Remove e retorna o waypoint de menor custo.
        heappop reorganiza o heap automaticamente.
        """
        if not self.is_empty():
            return heapq.heappop(self.heap)
        return None

    def is_empty(self):
        """Verifica se a fila de prioridade está vazia."""
        return len(self.heap) == 0

    def mostrar(self):
        """Exibe os itens do heap com print."""
        print(f"  [FILA PRIORIDADE] Conteúdo atual ({len(self.heap)} itens):")
        if self.heap:
            for custo, wp_id in sorted(self.heap):
                print(f"    custo={custo:.1f} | waypoint_id={wp_id}")
        else:
            print("    (fila de prioridade vazia)")
        return self.heap


# =============================================================================
# Usada para localizar um waypoint pelo seu ID na lista ordenada.
# Complexidade: O(log n) — mais eficiente que busca linear.
# =============================================================================

def busca_binaria_waypoint(nos_ordenados: list, id_alvo: int) -> dict:
    """
    Localiza um waypoint pelo ID usando Busca Binária.

    Pré-requisito: a lista deve estar ordenada por ID.
    A lista de waypoints já é criada em ordem crescente de ID,
    satisfazendo essa condição.

    Passos principais:
        1. Define limites l (inferior) e h (superior)
        2. Calcula m = (l + h) // 2
        3. Compara nos_ordenados[m]['id'] com id_alvo
        4. Descarta a metade que não contém o alvo
        5. Repete até encontrar ou esgotar o espaço de busca

    Args:
        nos_ordenados: lista de waypoints ordenada por ID
        id_alvo      : ID do waypoint a localizar

    Returns:
        dict com os metadados do waypoint, ou None se não encontrado
    """
    l, h = 0, len(nos_ordenados) - 1
    iteracao = 0

    while l <= h:
        iteracao += 1
        m = (l + h) // 2

        if nos_ordenados[m]["id"] == id_alvo:
            return nos_ordenados[m]
        elif nos_ordenados[m]["id"] < id_alvo:
            l = m + 1   # alvo na metade direita
        else:
            h = m - 1   # alvo na metade esquerda

    return None  # waypoint não encontrado


# =============================================================================
# GRAFO LUNAR — Grade 6x6 de Waypoints
# =============================================================================

def criar_grafo_lunar() -> dict:
    """
    Cria o grafo do Polo Sul Lunar com 36 waypoints em grade 6x6.

    Mapa de risco:
        Col→  0    1    2    3    4    5
    Lin↓
     0  BASE [1]  [1]  [4]  [7] [10]  [7]
     1       [1]  [4]  [4]  [7] [10]  [4]
     2       [4]  [4]  [1]  [1]  [7]  [4]
     3       [7] [10]  [1]  [1]  [4]  [1]
     4       [7] [10]  [4]  [4]  [1]  [1]
     5      [10]  [7]  [7]  [4]  [1] [1] DESTINO

    [1]=BAIXO [4]=MÉDIO [7]=ALTO [10]=CRÍTICO
    """
    nos = [
        # --- Linha 0 ---
        {"id":  0, "coord": (0, 0), "risco":  1, "label": "BAIXO",   "nome": "BASE"},
        {"id":  1, "coord": (1, 0), "risco":  1, "label": "BAIXO",   "nome": "WP-01"},
        {"id":  2, "coord": (2, 0), "risco":  4, "label": "MEDIO",   "nome": "WP-02"},
        {"id":  3, "coord": (3, 0), "risco":  7, "label": "ALTO",    "nome": "WP-03"},
        {"id":  4, "coord": (4, 0), "risco": 10, "label": "CRITICO", "nome": "CRATERA-01"},
        {"id":  5, "coord": (5, 0), "risco":  7, "label": "ALTO",    "nome": "WP-05"},
        # --- Linha 1 ---
        {"id":  6, "coord": (0, 1), "risco":  1, "label": "BAIXO",   "nome": "WP-06"},
        {"id":  7, "coord": (1, 1), "risco":  4, "label": "MEDIO",   "nome": "WP-07"},
        {"id":  8, "coord": (2, 1), "risco":  4, "label": "MEDIO",   "nome": "WP-08"},
        {"id":  9, "coord": (3, 1), "risco":  7, "label": "ALTO",    "nome": "WP-09"},
        {"id": 10, "coord": (4, 1), "risco": 10, "label": "CRITICO", "nome": "CRATERA-02"},
        {"id": 11, "coord": (5, 1), "risco":  4, "label": "MEDIO",   "nome": "WP-11"},
        # --- Linha 2 ---
        {"id": 12, "coord": (0, 2), "risco":  4, "label": "MEDIO",   "nome": "WP-12"},
        {"id": 13, "coord": (1, 2), "risco":  4, "label": "MEDIO",   "nome": "WP-13"},
        {"id": 14, "coord": (2, 2), "risco":  1, "label": "BAIXO",   "nome": "WP-14"},
        {"id": 15, "coord": (3, 2), "risco":  1, "label": "BAIXO",   "nome": "WP-15"},
        {"id": 16, "coord": (4, 2), "risco":  7, "label": "ALTO",    "nome": "WP-16"},
        {"id": 17, "coord": (5, 2), "risco":  4, "label": "MEDIO",   "nome": "WP-17"},
        # --- Linha 3 ---
        {"id": 18, "coord": (0, 3), "risco":  7, "label": "ALTO",    "nome": "WP-18"},
        {"id": 19, "coord": (1, 3), "risco": 10, "label": "CRITICO", "nome": "CRATERA-03"},
        {"id": 20, "coord": (2, 3), "risco":  1, "label": "BAIXO",   "nome": "WP-20"},
        {"id": 21, "coord": (3, 3), "risco":  1, "label": "BAIXO",   "nome": "WP-21"},
        {"id": 22, "coord": (4, 3), "risco":  4, "label": "MEDIO",   "nome": "WP-22"},
        {"id": 23, "coord": (5, 3), "risco":  1, "label": "BAIXO",   "nome": "WP-23"},
        # --- Linha 4 ---
        {"id": 24, "coord": (0, 4), "risco":  7, "label": "ALTO",    "nome": "WP-24"},
        {"id": 25, "coord": (1, 4), "risco": 10, "label": "CRITICO", "nome": "CRATERA-04"},
        {"id": 26, "coord": (2, 4), "risco":  4, "label": "MEDIO",   "nome": "WP-26"},
        {"id": 27, "coord": (3, 4), "risco":  4, "label": "MEDIO",   "nome": "WP-27"},
        {"id": 28, "coord": (4, 4), "risco":  1, "label": "BAIXO",   "nome": "WP-28"},
        {"id": 29, "coord": (5, 4), "risco":  1, "label": "BAIXO",   "nome": "WP-29"},
        # --- Linha 5 ---
        {"id": 30, "coord": (0, 5), "risco": 10, "label": "CRITICO", "nome": "CRATERA-05"},
        {"id": 31, "coord": (1, 5), "risco":  7, "label": "ALTO",    "nome": "WP-31"},
        {"id": 32, "coord": (2, 5), "risco":  7, "label": "ALTO",    "nome": "WP-32"},
        {"id": 33, "coord": (3, 5), "risco":  4, "label": "MEDIO",   "nome": "WP-33"},
        {"id": 34, "coord": (4, 5), "risco":  1, "label": "BAIXO",   "nome": "WP-34"},
        {"id": 35, "coord": (5, 5), "risco":  1, "label": "BAIXO",   "nome": "DESTINO"},
    ]

    COLS    = 6
    LINHAS  = 6
    adj     = {n["id"]: [] for n in nos}
    arestas = []

    direcoes = [(1, 0), (0, 1), (1, 1), (-1, 1)]

    for no in nos:
        nid       = no["id"]
        x, y      = no["coord"]
        r_origem  = no["risco"]

        for dx, dy in direcoes:
            nx, ny = x + dx, y + dy
            if 0 <= nx < COLS and 0 <= ny < LINHAS:
                nid_viz   = ny * COLS + nx
                r_destino = nos[nid_viz]["risco"]
                peso      = round((r_origem + r_destino) / 2, 1)

                arestas.append((nid, nid_viz, peso))
                arestas.append((nid_viz, nid, peso))

                if (nid_viz, peso) not in adj[nid]:
                    adj[nid].append((nid_viz, peso))
                if (nid, peso) not in adj[nid_viz]:
                    adj[nid_viz].append((nid, peso))

    return {"nos": nos, "arestas": arestas, "adj": adj}


# =============================================================================
# ALGORITMO DE DIJKSTRA
# =============================================================================

def dijkstra(adj: dict, origem: int, destino: int,
             nos: list, modo_degradado: bool = False) -> tuple:
    """
    Dijkstra com Heap Mínimo (FilaPrioridade).

    Complexidade:
        Tempo : O((V + E) log V)
        Espaço: O(V)

    Modo normal   : waypoints CRÍTICOS (risco=10) são bloqueados.
    Modo degradado: waypoints CRÍTICOS são aceitos com penalidade 1.5x (RN-04).
    """
    dist = {nid: float("inf") for nid in adj}
    prev = {nid: None for nid in adj}
    dist[origem] = 0

    fila_heap = FilaPrioridade()
    fila_heap.inserir(0, origem)

    historico_visita = PilhaLIFO()

    visitados = set()

    while not fila_heap.is_empty():
        custo_atual, u = fila_heap.remover_minimo()

        if u in visitados:
            continue

        visitados.add(u)
        historico_visita.empilhar(u)

        if u == destino:
            break

        for (v, peso_aresta) in adj[u]:
            if v in visitados:
                continue

            risco_v = nos[v]["risco"]

            if risco_v == RISCO_CRITICO:
                if not modo_degradado:
                    continue
                else:
                    peso_aresta = peso_aresta * PENALIDADE_GD

            novo_custo = custo_atual + peso_aresta

            if novo_custo < dist[v]:
                dist[v] = novo_custo
                prev[v] = u
                fila_heap.inserir(novo_custo, v)

    if dist[destino] == float("inf"):
        return float("inf"), []

    caminho = []
    atual   = destino
    while atual is not None:
        caminho.append(atual)
        atual = prev[atual]
    caminho.reverse()

    return round(dist[destino], 2), caminho


# =============================================================================
# FUNÇÃO PRINCIPAL
# =============================================================================

def calcular_rota_otimizada(grafo: dict, id_origem: int = 0,
                             id_destino: int = 35) -> dict:
    """
    Orquestra a otimização de rota do StellarPath AI:

        1. Busca Binária — localiza os waypoints de origem e destino
        2. Dijkstra + FilaPrioridade — calcula rota de menor risco
        3. FilaFIFO — organiza sequência de execução para o robô
        4. PilhaLIFO — backtracking em caso de bloqueio
        5. RN-04 (Graceful Degradation) — fallback quando rota normal falha
    """
    adj = grafo["adj"]
    nos = grafo["nos"]

    # --- PASSO 1: Busca Binária ---
    print("\n[BUSCA BINÁRIA] Localizando waypoints de origem e destino...")
    no_origem  = busca_binaria_waypoint(nos, id_origem)
    no_destino = busca_binaria_waypoint(nos, id_destino)

    if no_origem is None or no_destino is None:
        return {"status": "INALCANCAVEL", "custo_total": float("inf"),
                "caminho_ids": [], "caminho_nomes": [], "caminho_coords": [],
                "num_waypoints": 0, "distancia_km": 0.0,
                "nos_criticos_evitados": [], "modo_degradado": False}

    print(f"  Origem  encontrada: {no_origem['nome']}  (ID={no_origem['id']})")
    print(f"  Destino encontrado: {no_destino['nome']} (ID={no_destino['id']})")

    nos_criticos = [n["nome"] for n in nos if n["risco"] == RISCO_CRITICO]

    # --- PASSO 2: Dijkstra modo normal ---
    custo, caminho = dijkstra(adj, id_origem, id_destino, nos,
                               modo_degradado=False)
    modo_degradado = False
    status = "OK"

    # --- PASSO 3: Graceful Degradation com PilhaLIFO (RN-04) ---
    if custo == float("inf") or len(caminho) == 0:
        print("\n[AVISO RN-04] Rota normal bloqueada.")
        print("  PilhaLIFO: desfazendo waypoints visitados (backtracking)...")

        pilha_bt = PilhaLIFO()
        for wp in range(id_origem, id_destino + 1):
            pilha_bt.empilhar(wp)

        print("\n  Estado da pilha antes do backtracking:")
        pilha_bt.mostrar()

        while not pilha_bt.is_empty():
            pilha_bt.desempilhar()

        print("\n  Estado da pilha após o backtracking:")
        pilha_bt.mostrar()

        print("\n  Ativando Graceful Degradation — aceitando zonas CRÍTICAS...")
        custo, caminho = dijkstra(adj, id_origem, id_destino, nos,
                                   modo_degradado=True)
        modo_degradado = True
        status = "GRACEFUL_DEGRADATION"

    if custo == float("inf") or len(caminho) == 0:
        return {"status": "INALCANCAVEL", "custo_total": float("inf"),
                "caminho_ids": [], "caminho_nomes": [], "caminho_coords": [],
                "num_waypoints": 0, "distancia_km": 0.0,
                "nos_criticos_evitados": nos_criticos,
                "modo_degradado": modo_degradado}

    # --- PASSO 4: FilaFIFO — sequência de execução ---
    fila_execucao = FilaFIFO()
    mapa = {n["id"]: n for n in nos}

    print("\n[FILA FIFO] Carregando sequência de execução para o robô...")
    for nid in caminho:
        fila_execucao.enfileirar(mapa[nid]["nome"])

    print("\n  Estado da fila após carregar a rota:")
    fila_execucao.mostrar()

    print(f"\n  Próximo waypoint a executar (peek): {fila_execucao.peek()}")

    # --- PASSO 5: Métricas ---
    nomes_caminho  = [mapa[nid]["nome"]  for nid in caminho]
    coords_caminho = [mapa[nid]["coord"] for nid in caminho]

    distancia_total = 0.0
    for i in range(len(coords_caminho) - 1):
        x1, y1 = coords_caminho[i]
        x2, y2 = coords_caminho[i + 1]
        distancia_total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    return {
        "status"               : status,
        "custo_total"          : custo,
        "caminho_ids"          : caminho,
        "caminho_nomes"        : nomes_caminho,
        "caminho_coords"       : coords_caminho,
        "num_waypoints"        : len(caminho),
        "distancia_km"         : round(distancia_total, 2),
        "nos_criticos_evitados": nos_criticos,
        "modo_degradado"       : modo_degradado,
        "fila_execucao"        : fila_execucao,
    }


# =============================================================================
# FILA DE MISSÕES COM PRIORIDADE
# =============================================================================

class GerenciadorMissoes:
    """
    Gerenciador de missões lunares com fila de prioridade.

    Cada missão tem um nome e um nível de urgência (1 a 5).
    Missões de maior urgência são executadas primeiro.
    Em empate, segue a ordem de chegada (FIFO dentro da prioridade).

    Métodos:
        adicionar_missao  — enfileira uma missão
        executar_missao   — atende a de maior urgência
        mostrar_missoes   — exibe a fila atual com print
    """

    def __init__(self):
        """Construtor: cria a fila de missões vazia."""
        self.missoes = []

    def adicionar_missao(self, nome: str, urgencia: int, descricao: str = ""):
        """
        Adiciona uma missão à fila de prioridade.
        urgencia: 1 (menor) a 5 (maior).
        """
        self.missoes.append((nome, urgencia, descricao))
        print(f"  [MISSÃO] '{nome}' adicionada — urgência {urgencia}/5.")

    def executar_missao(self):
        """
        Executa a missão de maior urgência.
        Usa sort com key=lambda para ordenar
        por urgência decrescente, mantendo FIFO no empate.
        """
        if self.missoes:
            self.missoes.sort(key=lambda x: -x[1])
            missao = self.missoes.pop(0)
            print(f"  [EXECUÇÃO] Missão '{missao[0]}' — urgência {missao[1]}/5")
            if missao[2]:
                print(f"             Descrição: {missao[2]}")
            return missao
        else:
            print("  [AVISO] Nenhuma missão na fila.")
            return None

    def mostrar_missoes(self):
        """Exibe todas as missões pendentes com print."""
        print(f"  [GERENCIADOR] Missões pendentes ({len(self.missoes)}):")
        if self.missoes:
            ordenadas = sorted(self.missoes, key=lambda x: -x[1])
            for i, m in enumerate(ordenadas):
                print(f"    [{i + 1}] {m[0]:30s} | urgência {m[1]}/5"
                      + (f" | {m[2]}" if m[2] else ""))
        else:
            print("    (nenhuma missão pendente)")


# =============================================================================
# VISUALIZAÇÃO
# =============================================================================

def imprimir_mapa_risco(nos: list, caminho_ids: list) -> None:
    """
    Imprime o mapa 6x6 no terminal com a rota destacada.

    Legenda:
        [B]=BAIXO  [M]=MÉDIO  [A]=ALTO  [X]=CRÍTICO
        [*]=Rota   [S]=Base   [E]=Destino
    """
    COLS        = 6
    caminho_set = set(caminho_ids)
    simbolos    = {1: "B", 4: "M", 7: "A", 10: "X"}

    print("\n" + "=" * 55)
    print("  MAPA DE RISCO DO TERRENO LUNAR — StellarPath AI")
    print("  Legenda: [B]=Baixo [M]=Médio [A]=Alto [X]=Crítico")
    print("           [*]=Rota Otimizada [S]=Base [E]=Destino")
    print("=" * 55)

    print("     ", end="")
    for c in range(COLS):
        print(f"  {c} ", end="")
    print()
    print("     " + "----" * COLS)

    for linha in range(COLS):
        print(f"  {linha} |", end="")
        for col in range(COLS):
            nid     = linha * COLS + col
            n       = nos[nid]
            simbolo = simbolos.get(n["risco"], "?")

            if nid == 0:
                cell = "[S]"
            elif nid == 35:
                cell = "[E]"
            elif nid in caminho_set:
                cell = "[*]"
            else:
                cell = f"[{simbolo}]"

            print(f" {cell}", end="")
        print()

    print("     " + "----" * COLS)


def imprimir_relatorio(resultado: dict) -> None:
    """Imprime o relatório técnico completo da missão."""
    print("\n" + "=" * 55)
    print("  RELATÓRIO DE ROTA — StellarPath AI")
    print("=" * 55)

    print(f"\n  STATUS          : {resultado['status']}")
    print(f"  MODO DEGRADADO  : "
          f"{'SIM (RN-04 ativo)' if resultado['modo_degradado'] else 'NÃO'}")
    print(f"  CUSTO TOTAL     : {resultado['custo_total']} (risco acumulado)")
    print(f"  WAYPOINTS       : {resultado['num_waypoints']}")
    print(f"  DISTÂNCIA       : {resultado['distancia_km']} km")

    print(f"\n  ROTA CALCULADA (sequência FIFO para o robô):")
    for i, nome in enumerate(resultado["caminho_nomes"]):
        coord   = resultado["caminho_coords"][i]
        prefixo = ("  START →" if i == 0
                   else ("  → END  " if i == len(resultado["caminho_nomes"]) - 1
                         else f"     [{i:2d}]  →"))
        print(f"  {prefixo} {nome:15s} (x={coord[0]}, y={coord[1]})")

    print(f"\n  ZONAS CRÍTICAS EVITADAS ({len(resultado['nos_criticos_evitados'])}):")
    for nc in resultado["nos_criticos_evitados"]:
        print(f"           ⚠  {nc}")

    print("\n" + "=" * 55)


def simular_bloqueio_emergencia(grafo: dict) -> None:
    """
    Simula cenário de emergência (RN-04):
    bloqueia o corredor seguro e força Graceful Degradation.
    """
    print("\n" + "=" * 55)
    print("  SIMULAÇÃO RN-04 — GRACEFUL DEGRADATION")
    print("  Corredor principal bloqueado por tempestade solar")
    print("=" * 55)

    nos_bloqueados    = [14, 15, 20, 21]
    valores_originais = {}

    for nid in nos_bloqueados:
        valores_originais[nid]     = grafo["nos"][nid]["risco"]
        grafo["nos"][nid]["risco"] = RISCO_CRITICO
        grafo["nos"][nid]["label"] = "CRITICO"
        print(f"  [BLOQUEADO] {grafo['nos'][nid]['nome']} "
              f"→ risco elevado para CRÍTICO")

    print()
    resultado_gd = calcular_rota_otimizada(grafo, 0, 35)
    imprimir_relatorio(resultado_gd)

    for nid, valor in valores_originais.items():
        grafo["nos"][nid]["risco"] = valor
        grafo["nos"][nid]["label"] = "BAIXO" if valor == 1 else "MEDIO"


# =============================================================================
# PONTO DE ENTRADA
# =============================================================================

def main():
    print("\n" + "=" * 55)
    print("  STELLARPATH AI — Módulo de Otimização de Rota")
    print("  Polo Sul Lunar | Global Solution 2026 — FIAP")
    print("=" * 55)

    # --- Etapa 1: Criar grafo ---
    print("\n[1/4] Inicializando grafo do terreno lunar...")
    grafo = criar_grafo_lunar()
    total_nos     = len(grafo["nos"])
    total_arestas = len(set(
        (min(a, b), max(a, b)) for a, b, _ in grafo["arestas"]
    ))
    print(f"      Nós (waypoints) : {total_nos}")
    print(f"      Arestas únicas  : {total_arestas}")
    print(f"      Nós CRÍTICOS    : "
          f"{sum(1 for n in grafo['nos'] if n['risco'] == 10)}")
    print(f"      Nós BAIXO RISCO : "
          f"{sum(1 for n in grafo['nos'] if n['risco'] == 1)}")

    # --- Etapa 2: Rota otimizada ---
    print("\n[2/4] Calculando rota otimizada (modo normal)...")
    resultado = calcular_rota_otimizada(grafo, id_origem=0, id_destino=35)

    imprimir_mapa_risco(grafo["nos"], resultado["caminho_ids"])
    imprimir_relatorio(resultado)

    # --- Demonstração explícita dos métodos mostrar() ---
    print("\n--- DEMONSTRAÇÃO DAS ESTRUTURAS DE DADOS ---")

    print("\n  FilaFIFO — estado final da fila de execução:")
    resultado["fila_execucao"].mostrar()

    print("\n  PilhaLIFO — demonstração de empilhamento e estado:")
    pilha_demo = PilhaLIFO()
    for wp in ["BASE", "WP-06", "WP-14", "WP-20", "WP-21"]:
        pilha_demo.empilhar(wp)
    pilha_demo.mostrar()
    print(f"  Topo atual (peek): {pilha_demo.peek()}")

    print("\n  FilaPrioridade — estado do heap com 3 waypoints de exemplo:")
    fp_demo = FilaPrioridade()
    fp_demo.inserir(2.5, 14)
    fp_demo.inserir(1.0, 6)
    fp_demo.inserir(4.0, 21)
    fp_demo.mostrar()

    # --- Etapa 3: Fila de Missões com Prioridade ---
    print("\n[3/4] Gerenciando fila de missões com prioridade...")
    gerenciador = GerenciadorMissoes()
    gerenciador.adicionar_missao("Mapeamento Sísmico",    urgencia=3,
                                  descricao="Análise de estabilidade do solo")
    gerenciador.adicionar_missao("Reparo Solar Crítico",  urgencia=5,
                                  descricao="Painel danificado — risco de perda de energia")
    gerenciador.adicionar_missao("Coleta de Amostras",    urgencia=2,
                                  descricao="Coleta de regolito para análise")
    gerenciador.adicionar_missao("Calibração de Antenas", urgencia=4,
                                  descricao="Comunicação com estação terrestre")

    print()
    gerenciador.mostrar_missoes()
    print()
    gerenciador.executar_missao()
    gerenciador.executar_missao()
    print()
    print("  Fila após execução das duas missões prioritárias:")
    gerenciador.mostrar_missoes()

    # --- Etapa 4: Simulação Graceful Degradation ---
    print("\n[4/4] Simulando cenário de emergência (RN-04)...")
    simular_bloqueio_emergencia(grafo)

    print("\n  Módulo StellarPath AI encerrado com sucesso.")
    print("=" * 55)


if __name__ == "__main__":
    main()