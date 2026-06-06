# StellarPath AI — Módulo de Otimização de Rota Lunar

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![FIAP](https://img.shields.io/badge/FIAP-Global%20Solution%202026-red)

## Descrição

Módulo de navegação autônoma do projeto **StellarPath AI** para o Polo Sul Lunar.

Implementa o algoritmo de **Dijkstra com heap mínimo** sobre um grafo ponderado
de 36 waypoints, com suporte a **Graceful Degradation (RN-04)** para garantir
que o agente robótico nunca fique imobilizado em campo — mesmo quando o
corredor seguro principal é bloqueado por eventos externos como tempestades
de partículas solares.

Os pesos de risco de cada waypoint são derivados diretamente dos resultados
estatísticos do pipeline de Data Science do projeto (IQR e Z-Score) aplicados
sobre o dataset de crateras do Polo Sul Lunar
(Zenodo DOI: [10.5281/zenodo.5779260](https://doi.org/10.5281/zenodo.5779260)).

---

## Integrantes

| Nome | RM |
|---|---|
| Guilherme de Paula Kuskowski | RM562471 |
| Guilherme Eduardo de Lima | RM566045 |
| Enzo de Faria Ferreira | RM562448 |

---

## Estrutura do Projeto
stellarpath-ai/
├── stellarpath_rota.py   # Script principal — algoritmo completo
├── README.md             # Este arquivo
└── requirements.txt      # Sem dependências externas

---

## Como Executar

### Pré-requisitos

- Python 3.8 ou superior
- Nenhuma dependência externa — utiliza apenas `heapq` e `math` da biblioteca padrão

### Instalação

```bash
git clone https://github.com/seu-usuario/stellarpath-ai.git
cd stellarpath-ai
```

### Execução

```bash
python stellarpath_rota.py
```

---

## O que o script executa

O programa roda automaticamente em **4 etapas**:

**[1/4] Inicialização do grafo**
Cria os 36 waypoints em grade 6×6 com seus pesos de risco e constrói
a lista de adjacência com conectividade 4-direcional e diagonal.

**[2/4] Cálculo da rota otimizada (modo normal)**
Executa a Busca Binária para localizar origem e destino, depois aplica
Dijkstra com FilaPrioridade (heap mínimo) para encontrar o caminho de
menor risco acumulado. Carrega a rota em uma FilaFIFO para execução
sequencial pelo robô e exibe o mapa 6×6 com a rota destacada.

**[3/4] Gerenciamento de missões por prioridade**
Demonstra o GerenciadorMissoes: adiciona 4 missões com diferentes
urgências (1 a 5), exibe a fila ordenada e executa as duas de maior
prioridade.

**[4/4] Simulação do modo Graceful Degradation (RN-04)**
Bloqueia o corredor seguro principal elevando 4 waypoints para risco
CRÍTICO, força o replanejamento com PilhaLIFO (backtracking) e
recalcula a rota aceitando zonas críticas com penalidade de 50%.

---

## Estruturas de Dados

| Estrutura | Uso no projeto |
|---|---|
| `FilaFIFO` | Sequência de execução de waypoints pelo robô |
| `PilhaLIFO` | Backtracking de rotas bloqueadas (RN-04) |
| `FilaPrioridade` | Heap mínimo interno do algoritmo de Dijkstra |
| `GerenciadorMissoes` | Fila de missões ordenada por urgência |
| `busca_binaria_waypoint` | Localização de waypoints por ID em O(log n) |

---

## Grafo do Terreno Lunar
Col→   0    1    2    3    4    5
Lin↓
0  BASE  [1]  [1]  [4]  [7] [10]  [7]
1        [1]  [4]  [4]  [7] [10]  [4]
2        [4]  [4]  [1]  [1]  [7]  [4]
3        [7] [10]  [1]  [1]  [4]  [1]
4        [7] [10]  [4]  [4]  [1]  [1]
5       [10]  [7]  [7]  [4]  [1]  [1]  DESTINO
[1]=BAIXO   [4]=MÉDIO   [7]=ALTO   [10]=CRÍTICO

- **36 waypoints** em grade 6×6
- **5 crateras** classificadas como CRÍTICO (risco = 10) — bloqueadas em modo normal
- **Conectividade**: 4 direções + diagonal (até 4 vizinhos por nó)
- **Peso da aresta**: média dos riscos dos dois nós conectados

---

## Complexidade Algorítmica

| Componente | Complexidade |
|---|---|
| Criação do grafo | O(V × D) — V vértices, D direções |
| Busca Binária | O(log V) |
| Dijkstra com heap | **O((V + E) log V)** |
| Reconstrução do caminho | O(V) |
| **Total** | **O((V + E) log V)** |

Com V = 36 e E ≈ 100, a solução responde em microssegundos —
compatível com hardware embarcado de navegação espacial em tempo real.

---

## Regras de Negócio Implementadas

| Regra | Descrição |
|---|---|
| **RN-04** | Graceful Degradation: quando todos os caminhos normais estão bloqueados, aceita nós CRÍTICOS com penalidade de 1.5× no custo |
| **IQR — CRÍTICO** | Nós com risco = 10 (outlier extremo, fator 3.0) bloqueiam a rota em modo normal |
| **IQR — ALTO** | Nós com risco = 7 (outlier moderado, fator 1.5) são aceitos com custo elevado |

---

## Referência do Dataset

- **Nome:** South Polar Small and Medium Craters (sem PSR)
- **DOI:** [10.5281/zenodo.5779260](https://doi.org/10.5281/zenodo.5779260)
- **Autor:** Ramiro Marco Figuera — Jacobs University Bremen (2021)
- **Limiares CRÍTICO:** diâmetro > 0,3619 km | profundidade > 0,0922 km | inclinação > 31,37°

---

## Licença

MIT License — consulte o arquivo LICENSE para detalhes.
